/**
 * Created by Administrator on 2016/4/19.
 * 整个应用的WebSocket服务，这个服务用来与后台进行长时间通信
 */


import { Injectable } from '@angular/core';
// import {Observable} from 'rxjs/Observable'
import { Subject } from 'rxjs/Subject'
import { BehaviorSubject } from 'rxjs/BehaviorSubject'
import * as io from 'socket.io-client'
import {
  JY901Data,
  ArdiunoData,
  OdomData,
} from './models'


@Injectable()
export class WebSocketService {
  private _socket  // 连接服务端的WebSocket
  public loginUser$: BehaviorSubject<string> = new BehaviorSubject('nobody')
  // public loginUser$: BehaviorSubject<string> = new BehaviorSubject('admin')

  /**
   * 连接服务器WebSocket成功或断开
   * @type {Subject}
   */
  public connected$: Subject<any> = new Subject<any>()
  /**
   * 向服务器发送登录登出事件并获取返回值
   * @type {Subject}
   */
  public loginResult$: Subject<{succeeded: boolean, username: string, action: string}>
    = new Subject<{succeeded: boolean, username: string, action: string}>()
  /**
   * 当前是否已经登录成功
   * @type {Subject<boolean>}
   */
  public isAuth$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(false)
  /**
   * 从服务器获取得的图片数据
   * @type {Subject}
   */
  public image$: Subject<ArrayBuffer> = new Subject<ArrayBuffer>()
  /**
   * 从服务器获取得的JY901传感器数据
   * @type {Subject}
   */
  public jy901$: Subject<JY901Data> = new Subject<JY901Data>()
  /**
   * 从服务器获取得的arduino的传感器数据
   * @type {Subject}
   */
  public arduino$: Subject<ArdiunoData> = new Subject<ArdiunoData>()
  /**
   * 从服务器获取得的融合滤波后的里程计数据
   * @type {Subject}
   */
  public eo$: Subject<OdomData> = new Subject<OdomData>()
  /**
   * 从服务器获取得的轮式里程计数据
   * @type {Subject}
   */
  public wo$: Subject<OdomData> = new Subject<OdomData>()
  /**
   * 从服务器获取得的惯导里程计数据
   * @type {Subject}
   */
  public io$: Subject<OdomData> = new Subject<OdomData>()
  /**
   * 从服务器获取得的轮式 + 惯导里程计数据
   * @type {Subject}
   */
  public to$: Subject<OdomData> = new Subject<OdomData>()
  /**
   * 从服务器获取得的视觉里程计数据
   * @type {Subject}
   */
  public vo$: Subject<OdomData> = new Subject<OdomData>()

  constructor () {
    this.connect()
  }

  /**
   * 连接连接服务端WebSocket，并初始化该客户端WebSocket要监听的服务器端WebSocket发送过来的一系列事件
   */
  connect () {
    const host = ENV === 'development' 
      ? 'ws://192.168.66.108:61620' 
      : `ws://${location.host}`
    console.log(host)
    this._socket = io(host)

    this._socket
      .on('connect', () => {  // 连接成功
        this.connected$.next(true)

        this._socket
          .on('welcome', (data: {welcome: string}) => {
            // 服务端发送来的welcome事件
            console.log(data.welcome)

          })
          .on('login_rs', (data: {succeeded: boolean, username: string, action: string}) => {
            // 服务端响应客户端发送的login事件并返回的login_rs事件
            if (data.succeeded) {
              this.loginUser$.next(data.username)
            }

            this.loginResult$.next(data)
            this.isAuth$.next(data.succeeded)
          })
          .on('logout_rs', (data: {succeeded: boolean, username: string, action: string}) => {
            // 服务端响应客户端发送的logout事件并返回的logout_rs事件
            this.loginUser$.next('nobody')
            this.loginResult$.next(data)
            this.isAuth$.next(false)
          })
          .on('jy901', (data: {jy901Data: string}) => {  // JY901传感器数据
            const dataStrings = data.jy901Data.split(' ')  // web服务器传来的数据是用空格隔开的
            this.jy901$.next({
              ax: parseFloat(dataStrings[0]),
              ay: parseFloat(dataStrings[1]),
              az: parseFloat(dataStrings[2]),  // 加速度
              wx: parseFloat(dataStrings[3]),
              wy: parseFloat(dataStrings[4]),
              wz: parseFloat(dataStrings[5]),  // 角速度
              pitch: parseFloat(dataStrings[6]),
              roll: parseFloat(dataStrings[7]),
              yaw: parseFloat(dataStrings[8]),  // 角度
              temperature: parseFloat(dataStrings[9]),  // 温度
            })

          })
          .on('arduino', (data: {type: string, arduinoData: string}) => {  // arduino上的传感器数据
            if (data.type === '\x83') {
              return
            }
            const dataStrings = data.arduinoData.split(' ')  // web服务器传来的数据是用空格隔开的
            this.arduino$.next({
              HUMI: parseFloat(dataStrings[0]),
              TEMP: parseFloat(dataStrings[1]),  // 温湿度
              warning_down_left_front: dataStrings[2] === '1',
              warning_down_right_front: dataStrings[3] === '1',
              warning_down_right_back: dataStrings[4] === '1',
              warning_down_left_back: dataStrings[5] === '1',  // 向下的红外，true说明远离地面
              warning_front_left_front: dataStrings[6] === '1',
              warning_front_right_front: dataStrings[7] === '1',
              warning_front_right_back: dataStrings[8] === '1',
              warning_front_left_back: dataStrings[9] === '1',  // 向前的红外，true说明前方有障碍
              dist0: parseFloat(dataStrings[10]),
              dist1: parseFloat(dataStrings[11]),
              dist2: parseFloat(dataStrings[12]),
              dist3: parseFloat(dataStrings[13]),  // 超声波
              infraredShow: {
                down: dataStrings[2] + dataStrings[3] + dataStrings[4] + dataStrings[5],
                front: dataStrings[6] + dataStrings[7] + dataStrings[8] + dataStrings[9]
              }
            })

          })
          .on('image', (data: {image: ArrayBuffer}) => {  // 接收得到的是ArrayBuffer对象
            // 服务端发送图片过来
            this.image$.next(data.image)
          })
          .on('info', (data: {type: string, infoData: string}) => {  // 小车计算数据
            const dataStrings = data.infoData.split(' ')
            const infoPackage = {
              vx: parseFloat(dataStrings[0]),
              vy: parseFloat(dataStrings[1]),
              vz: parseFloat(dataStrings[2]),  // 加速度
              wx: parseFloat(dataStrings[3]),
              wy: parseFloat(dataStrings[4]),
              wz: parseFloat(dataStrings[5]),  // 角速度
              pitch: parseFloat(dataStrings[6]),
              roll: parseFloat(dataStrings[7]),
              yaw: parseFloat(dataStrings[8]),  // 角度
              x: parseFloat(dataStrings[9]),
              y: parseFloat(dataStrings[10]),
              z: parseFloat(dataStrings[11]),  // 位置
            }
            switch (data.type) {
              case 'eo': {
                this.eo$.next(infoPackage)
              }
                break
              case 'wo': {
                this.wo$.next(infoPackage)
              }
                break
              case 'io': {
                this.io$.next(infoPackage)
              }
                break
              case 'to': {
                this.to$.next(infoPackage)
              }
                break
              case 'vo': {
                this.vo$.next(infoPackage)
              }
                break
            }
          })

      })
      .on('disconnect', () => {
        this.connected$.next(false)
      })

  }

  /**
   * 向后台请求获取图像
   */
  imageOn () {
    this._socket.emit('imageOn', {})
  }

  /**
   * 向后台提示不需要图像了
   */
  imageOff () {  
    this._socket.emit('imageOff', {})
  }

  /**
   * 发送电机控制命令参数给后台
   * @param motorLeftSpeed 要改变的左电机速度
   * @param motorRightSpeed 要改变的右电机速度
   */
  motorsControl (motorLeftSpeed: number, motorRightSpeed: number) {
    this._socket.emit('motorsControl', {
      motorLeftSpeed: motorLeftSpeed,
      motorRightSpeed: motorRightSpeed
    })
  }

  /**
   * 发送固定电机运动命令参数给后台
   * @param type 圆形运动等等
   */
  sendFixMotorsControl(type: string) {
    this._socket.emit(type, {})
  }

  /**
   * 登录，向服务端WebSocket发送login事件
   */
  login (username: string, password: string) {
    this._socket.emit('login', {username, password})
  }

  /**
   * 登出
   */
  logout () {
    this._socket.emit('logout', {username: this.loginUser$})
  }

  /**
   * 重置位姿
   */
  restartPosition () {

  }
}
