/**
 * Created by Alvin.liu on 2016/11/26.
 * 方向盘控制
 */


import { Component, OnInit, OnDestroy } from '@angular/core'
import {
  WebSocketService,
  MotorService
} from  '../../../service-share/services'
import { MotorState } from '../../../service-share/services/motor'
// import {Subject} from 'rxjs/Subject'


@Component({
  selector: 'steering-wheel',
  styles: [require('./steering-wheel.component.scss')],
  template: require('./steering-wheel.component.html')

})
export class SteeringWheelComponent implements OnInit, OnDestroy {

  private CONTROL_MODE: string = 'key' // 控制模式
  private _updateTimerID  // 更新_flag的定时器id，随着按下按钮的时间长度增大电机的速度
  private _keyDownHandle  // 给document的keydown事件绑定的处理函数
  private _keyUpHandle  // 给document的keyup事件绑定的处理函数
  private _touchStartHandle  // 给document的touchstart事件绑定的处理函数
  private _touchEndHandle  // 给document的touchend事件绑定的处理函数

  /**
   * 键盘方向盘
   */

  private _upPress: boolean = false
  private _downPress: boolean = false
  private _leftPress: boolean = false
  private _rightPress: boolean = false
  private _flag: number = 0b0000  // 用二进制记录当前上下左后按钮被按下的状况

  constructor (private _wsService: WebSocketService,
               private _mtService: MotorService) {

  }

  ngOnInit () {
    // 添加一个timer定时执行更新motors操作
    this._updateTimerID = setInterval(() => this.controlMotors(), 50)

    // 使用箭头函数保证this指向本实例
    this._keyDownHandle = (event: KeyboardEvent) => {
      this._onKeyDown(event)
    }
    this._keyUpHandle = (event: KeyboardEvent) => {
      this._onKeyUp(event)
    }
    this._touchStartHandle = (event: TouchEvent) => {
      this._onTouchStart(event)
    }
    this._touchEndHandle = (event: TouchEvent) => {
      this._onTouchEnd(event)
    }

    // 绑定PC端键盘事件
    document.addEventListener('keydown', this._keyDownHandle)
    document.addEventListener('keyup', this._keyUpHandle)
    // 绑定移动端触摸事件
    document.addEventListener('touchstart', this._touchStartHandle)
    document.addEventListener('touchend', this._touchEndHandle)

  }

  ngOnDestroy () {  // 取消timer、键盘事件和触摸事件
    clearInterval(this._updateTimerID)
    document.removeEventListener('keydown', this._keyDownHandle)
    document.removeEventListener('keyup', this._keyUpHandle)
    document.removeEventListener('touchstart', this._touchStartHandle)
    document.removeEventListener('touchend', this._touchEndHandle)
  }

  /**
   * 使用MotorService根据_flag不断更新电机速度，如果速度值更改，
   * 则使用WebSocketService发送命令给后台让后台控制电机
   */
  private controlMotors () {
    if (this._mtService.changeSpeed(this.CONTROL_MODE, this._flag)) {
      this._wsService.motorsControl(this.motorLeftSpeed, this.motorRightSpeed)
    }

  }

  /**
   * 移动端的触摸事件，一个时间段可能有多个触摸点
   * @param event
   * @returns {boolean}
   */
  private _onTouchStart (event: TouchEvent) {

    if (event.srcElement instanceof HTMLSpanElement) {
      // 查看触碰的位置是否是方向盘按钮（span置于button前）
      switch (event.srcElement.id) {
        case "keyUpSpan": {
          this._flag |= 0b1000
          this._upPress = true
          event.preventDefault()
        }
          break
        case "keyDownSpan": {
          this._flag |= 0b0100
          this._downPress = true
          event.preventDefault()
        }
          break
        case "keyLeftSpan": {
          this._flag |= 0b0010
          this._leftPress = true
          event.preventDefault()
        }
          break
        case "keyRightSpan": {
          this._flag |= 0b0001
          this._rightPress = true
          event.preventDefault()
        }
          break
      }

    } else if (event.srcElement instanceof HTMLButtonElement) {
      // 查看触碰的位置是否是方向盘按钮
      switch (event.srcElement.id) {
        case "keyUpButton": {
          this._flag |= 0b1000
          this._upPress = true
          event.preventDefault()
        }
          break
        case "keyDownButton": {
          this._flag |= 0b0100
          this._downPress = true
          event.preventDefault()
        }
          break
        case "keyLeftButton": {
          this._flag |= 0b0010
          this._leftPress = true
          event.preventDefault()
        }
          break
        case "keyRightButton": {
          this._flag |= 0b0001
          this._rightPress = true
          event.preventDefault()
        }
          break
      }


    }

  }

  /**
   * 移动端的触摸结束事件，触摸结束时直接判断srcElement
   * @param event
   * @returns {boolean}
   */
  private _onTouchEnd (event: TouchEvent) {

    if (event.srcElement instanceof HTMLSpanElement) {
      // 查看触碰的位置是否是方向盘按钮（span置于button前）
      switch (event.srcElement.id) {
        case "keyUpSpan": {
          this._flag &= 0b0111
          this._upPress = false
        }
          break
        case "keyDownSpan": {
          this._flag &= 0b1011
          this._downPress = false
        }
          break
        case "keyLeftSpan": {
          this._flag &= 0b1101
          this._leftPress = false
        }
          break
        case "keyRightSpan": {
          this._flag &= 0b1110
          this._rightPress = false
        }
          break
      }

    } else if (event.srcElement instanceof HTMLButtonElement) {
      switch (event.srcElement.id) {
        case "keyUpButton": {
          this._flag &= 0b0111
          this._upPress = false
        }
          break
        case "keyDownButton": {
          this._flag &= 0b1011
          this._downPress = false
        }
          break
        case "keyLeftButton": {
          this._flag &= 0b1101
          this._leftPress = false
        }
          break
        case "keyRightButton": {
          this._flag &= 0b1110
          this._rightPress = false
        }
          break
      }

    }
  }

  /**
   * 按键按下则记录按下状态
   * @param event
   */
  private _onKeyDown (event: KeyboardEvent) {
    switch (event.keyCode) {
      case 87:  // w
      {
        this._flag |= 0b1000
        this._upPress = true
      }
        break
      case 83:  // s
      {
        this._flag |= 0b0100
        this._downPress = true
      }
        break
      case 65:  // a
      {
        this._flag |= 0b0010
        this._leftPress = true
      }
        break
      case 68:  // d
      {
        this._flag |= 0b0001
        this._rightPress = true
      }
        break
      case 38:  // 上
      {
        this._flag |= 0b1000
        this._upPress = true
      }
        break
      case 40:  // 下
      {
        this._flag |= 0b0100
        this._downPress = true
      }
        break
      case 37:  // 左
      {
        this._flag |= 0b0010
        this._leftPress = true
      }
        break
      case 39:  // 右
      {
        this._flag |= 0b0001
        this._rightPress = true
      }
        break
    }
  }

  /**
   * 按键弹起则记录未按下状态
   * @param event
   */
  private _onKeyUp (event: KeyboardEvent) {
    switch (event.keyCode) {
      case 87:  // w
      {
        this._flag &= 0b0111
        this._upPress = false
      }
        break
      case 83:  // s
      {
        this._flag &= 0b1011
        this._downPress = false
      }
        break
      case 65:  // a
      {
        this._flag &= 0b1101
        this._leftPress = false
      }
        break
      case 68:  // d
      {
        this._flag &= 0b1110
        this._rightPress = false
      }
        break
      case 38:  // 上
      {
        this._flag &= 0b0111
        this._upPress = false
      }
        break
      case 40:  // 下
      {
        this._flag &= 0b1011
        this._downPress = false
      }
        break
      case 37:  // 左
      {
        this._flag &= 0b1101
        this._leftPress = false
      }
        break
      case 39:  // 右
      {
        this._flag &= 0b1110
        this._rightPress = false
      }
        break
    }
  }

  /**
   * 显示_flag的字符串
   * @returns {string}
   */
  get flag () {
    let res = ''
    if (this._upPress) res += '1'
    else res += '0'
    if (this._downPress) res += '1'
    else res += '0'
    if (this._leftPress) res += '1'
    else res += '0'
    if (this._rightPress) res += '1'
    else res += '0'
    return res
  }

  get titleShow () {
    return `L: ${this.motorLeftSpeed}; R: ${this.motorRightSpeed}; ${this.motorState}; ${this.flag}`
  }

  get motorLeftSpeed () {
    return this._mtService.motorLeftSpeed
  }

  get motorRightSpeed () {
    return this._mtService.motorRightSpeed
  }

  get motorState () {
    return MotorState[this._mtService.motorState]
  }
}
