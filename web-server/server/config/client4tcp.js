/**
 * Created by alvin.liu on 2016/4/29.
 * 这里我们的web服务器与小车tcp服务器的关系是客户端与服务器的关系，即我们的web服务器作为客户端来连接真实的小车服务器，
 * 而浏览器客户端通过web服务器这个代理来获取小车的信息或控制小车。所以在此处我们需要构造tcp连接去连接真实的小车服务器
 * 即本web服务进程即为TCP小车服务器的admin客户端
 *
 * 由于web服务器与小车tcp服务器在同一个设备中部署，所以直接连接本地端口
 * 当连接上小车服务器后，本进程作为一个客户端，立即获取管理员权限，获取小车数据和图像，
 * 之后使用Rx库，当有数据到达时，立即广播到订阅了这些数据的客户端（websocket）
 *
 * TCP server的端口:
 * 1、61611端口接受client的控制命令以发送电机串口命令控制小车；（现在直接发送到串口，以后应该又ROS获取这些命令来进行发布主题）
 * 2、61612端口接受client的连接并持续向client发送jy901陀螺仪信息；
 * 3、61613端口接受client的连接并持续向client发送arduino信息；
 * 4、61614端口接受client的连接并持续向client发送小车的图像数据；
 * 5、61615端口接手client的连接并持续向client发送小车的运算结果；
 *
 */

import * as net from 'net'
import { env } from './env'
import { logger } from './log'

const directive = require('./directive')

/**
 * 控制
 */

let controlClient
function connectControl () {
  controlClient = net.connect(61611, env.tcp_host, () => {
    logger.info('connect 61611 control server')
    controlClient.write('#admin:gxnu#')  // 一连接就登录


  })
    .setEncoding('binary')
    .on('data', (data) => {  // 接收服务器发过来的数据（控制命令的反馈）
      logger.info(data.toString())
    })
    .on('end', () => {
      logger.info('disconnected from 61611 control server')
      connectControl()
    })
    .on('error', (error) => {
      // logger.error(error)
      connectControl()
    })
}
connectControl()

export function sendControlCommand (type, parameters) {  // 发送控制命令
  let cmd = ''
  switch (type) {
    case 'motorsControl': {
      cmd = directive.makeControlMotorCommand(parameters.motorLeftSpeed, parameters.motorRightSpeed)
      // logger.info(Buffer.from(cmd, 'binary'))
      controlClient.write(cmd, 'binary')  // 要以二进制方式发送字符串，否则使用默认编码会出错
    }
    case 'motorsCircle': {
      cmd = directive.makeControlMotorCommand(20, 50)
      // logger.info(Buffer.from(cmd, 'binary'))
      controlClient.write(cmd, 'binary')  // 要以二进制方式发送字符串，否则使用默认编码会出错
    }
  }
}


/**
 * 数据
 */

// 61612 jy901
let jy901Client
function connectJy901 () {
  jy901Client = net.connect(61612, env.tcp_host, () => {
    logger.info('connect 61612 jy901 server')
    jy901Client.write('#admin:gxnu#')  // 一连接就登录


  })
    .setEncoding('binary')
    .on('data', (data) => {  // 接收服务器发过来的数据
      directive.parseData(data)

    })
    .on('end', () => {
      logger.info('disconnected from 61612 jy901 server')
      connectJy901()
    })
    .on('error', (error) => {
      // logger.error(error)
      connectJy901()
    })
}
connectJy901()

// 61613 arduino
let arduinoClient
function connectArduino () {
  arduinoClient = net.connect(61613, env.tcp_host, () => {
    logger.info('connect 61613 arduino server')
    arduinoClient.write('#admin:gxnu#')  // 一连接就登录


  })
    .setEncoding('binary')
    .on('data', (data) => {  // 接收服务器发过来的数据
      directive.parseData(data)

    })
    .on('end', () => {
      logger.info('disconnected from 61613 arduino server')
      connectArduino()
    })
    .on('error', (error) => {
      // logger.error(error)
      connectArduino()
    })
}
connectArduino()

// 61614 image
let imageClient
function connectImage () {
  imageClient = net.connect(61614, env.tcp_host, () => {
    logger.info('connect 61614 image server')
    imageClient.write('#admin:gxnu#')  // 一连接就登录

  })
    .setEncoding('binary')
    .on('data', (data) => {  // 接收服务器发过来的图像数据
      // 解析并由directive.imageProxy发射图片流
      directive.parseImage(data)
    })
    .on('end', () => {
      logger.info('disconnected from 61614 image server')
      connectImage()
    })
    .on('error', (error) => {
      // logger.error(error)
      connectImage()
    })
}
connectImage()

// 61615 infoClient
let infoClient
function connectInfo () {
  infoClient = net.connect(61615, env.tcp_host, () => {
    logger.info('connect 61615 info server')
    infoClient.write('#admin:gxnu#')  // 一连接就登录


  })
    .setEncoding('binary')
    .on('data', (data) => {  // 接收服务器发过来的数据
      directive.parseData(data)

    })
    .on('end', () => {
      logger.info('disconnected from 61615 info server')
      connectInfo()
    })
    .on('error', (error) => {
      // logger.error(error)
      connectInfo()
    })
}
connectInfo()