/**
 * Created by alvin.liu on 2016/4/26.
 * 电机服务，保存电机的速度
 */

import { Injectable } from '@angular/core'
import { DualMotor } from './motor'
import {
  WebSocketService,
} from  '../websocket'

/**
 * MotorService会在调用changeSpeed时改变电机速度
 */
@Injectable()
export class MotorService {

  private _canControl: boolean = true

  private preMotorLeftSpeed
  private preMotorRightSpeed
  private _motors: DualMotor

  constructor (private _wsService: WebSocketService) {
    this._motors = new DualMotor()
  }

  runCircle () {
    this._canControl = false
    this._wsService.sendFixMotorsControl('motorsCircle')
  }

  stopCircle () {
    this._canControl = true
    this._wsService.motorsControl(0, 0)
  }
  
  /**
   * 根据方向盘flag更改motors的speed
   * @param mode "key"，"drag"
   * @param flag 如果mode="key"则flag为方向按键标识，4bit二进制分别对应wsad，
   * 如果是"drag"，则为长度归一化的极坐标点 [theta, length]
   * @returns {boolean}电机速度较之前已经有了更改，指明需要发送命令到后台
   */
  changeSpeed (mode: string, flag: any): boolean {
    if (!this._canControl) {
      return
    }
    this.preMotorLeftSpeed = this.motorLeftSpeed
    this.preMotorRightSpeed = this.motorRightSpeed
    if (mode == 'key')
      this._motors.changeSpeedByKey(flag)
    else if (mode == 'drag')
      this._motors.changeSpeedByDrag(flag)
    
    if (this.preMotorLeftSpeed != this.motorLeftSpeed || this.preMotorRightSpeed != this.motorRightSpeed) {
      return
    }

    this._wsService.motorsControl(this.motorLeftSpeed, this.motorRightSpeed)
  }

  changeSpeedToZero (mode: string) {
    if (!this._canControl) {
      return false
    }
    if (mode == 'key')
      this._motors.changeSpeedByKey(0b0000)
    else if (mode == 'drag')
      this._motors.changeSpeedByDrag([0, 0])
    
    if (this.preMotorLeftSpeed != this.motorLeftSpeed || this.preMotorRightSpeed != this.motorRightSpeed) {
      return
    }
    
    this._wsService.motorsControl(this.motorLeftSpeed, this.motorRightSpeed)
  }

  get motorLeftSpeed (): number {
    return this._motors.motorLeft.speed
  }

  get motorRightSpeed (): number {
    return this._motors.motorRight.speed
  }

  get motorState () {
    return this._motors.curMotorState
  }
}
