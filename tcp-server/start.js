/**
 * Created by Alvin Liu on 2016/4/27.
 * 启动TCP服务器，这里将会打开5个端口来进行服务：
 * 1、61611端口接受client的控制命令以发送电机串口命令控制小车；（现在直接发送到串口，以后应该又ROS获取这些命令来进行发布主题）
 * 2、61612端口接受client的连接并持续向client发送jy901陀螺仪信息；
 * 3、61613端口接受client的连接并持续向client发送arduino信息；
 * 4、61614端口接受client的连接并持续向client发送小车的图像数据；
 * 5、61615端口接受client的连接并持续向client发送小车的运算结果（包括eo、vo、wo）；
 * 所以一个完整的小车服务器将会有多个TCP服务和一个web服务（61620端口），
 *
 * 而小车客户端，则视情况而定来连接5个服务器以获取数据
 *
 * 这里的客户端有以下几种情况：
 *
 * 1、python进程，
 * 该进程会有多个，其中会有两个python进程通过连接服务器获取数据再通过ROS发布传感器主题（陀螺仪，arduino）
 * 会有一个python进程获取图像，并连接图像服务器，发送图像数据，同时通过ROS发布图像主题
 * 会有运算核心连接运算结果服务器，发送结果数据
 *
 * 2、web服务器：
 * web服务器会以客户端身份连接进来（admin身份登录）以控制小车与获取数据
 *
 */

'use strict'

const controlServer = require('./control_server').controlServer
const jy901Server = require('./jy901_server').jy901Server
const arduinoServer = require('./arduino_server').arduinoServer
const imageServer = require('./image_server').imageServer
const infoServer = require('./info_server').infoServer
