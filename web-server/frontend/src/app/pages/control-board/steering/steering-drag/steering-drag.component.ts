/**
 * Created by Alvin.liu on 2016/11/26.
 * 拖拽控制
 */

import { Component, OnInit, OnDestroy } from '@angular/core'
import {
  WebSocketService,
  MotorService
} from '../../../../service-share/services'
import { MotorState } from '../../../../service-share/services/motor'
declare var fabric

@Component({
  selector: 'steering-drag',
  styles: [require('./steering-drag.component.scss')],
  template: require('./steering-drag.component.html')
})
export class SteeringDragComponent implements OnInit, OnDestroy {

  private CONTROLMODE: string = 'drag'  // 控制模式
  private _updateTimerID  // 更新_flag的定时器id，随着拖拽的位置更新电机的速度

  private _flag: [number, number] = [0, 0]  // 拖拽点相对于圆中心的长度归一化的极坐标 [theta, length]

  /**
   * 拖拽方向盘，一个用fabric.js绘制的的画布对象
   */

  private _dragSteering
  private _wheelHandle  // 方向盘的控制柄
  private _wheelHandlePress: boolean = false  // 控制柄是否被按下
  private _wheelCenter: [number, number] = [0, 0]  // 方向盘中心应该放置的位置（以左顶点）
  private _wheelRadius: number  // 方向盘图片的半径

  constructor(private _wsService: WebSocketService,
    private _mtService: MotorService) {

  }

  ngOnInit() {
    // 添加一个timer定时执行更新motors操作，使用箭头函数保证this指向本实例
    this._updateTimerID = setInterval(() => this.controlMotors(), 50)
    this._initDragSteering()

  }

  ngOnDestroy() {  // 取消timer
    clearInterval(this._updateTimerID)
  }

  private _initDragSteering() {
    /**
     * 初始化拖拽方向盘
     */

    this._dragSteering = new fabric.Canvas('canvas-drag', {
      backgroundColor: 'rgba(255, 255, 255, 0)',
      selection: false
      // ...
    })
    // this._dragSteering.getElement  // HTMLCanvasElement
    // this._dragSteering.getContext()  // canvas context 2d

    //  设置圆盘背景
    // fabric.Image.fromURL('./assets/images/round.jpg', (img) => {
    //   img.set({ left: 0, top: 0, angle: 0 }).scale(this._dragSteering.width / img.width)
    //   this._dragSteering.setBackgroundImage(img, () => {
    //     this._dragSteering.renderAll()
    //   })
    // })
    this._wheelRadius = this._dragSteering.width / 2 - 30

    // 设置圆盘
    const circle = new fabric.Circle({
      radius: this._wheelRadius + 20,
      fill: '',
      stroke: 'red',
      strokeWidth: 3,
      selectable: false
    })
    circle.setLeft(this._dragSteering.width / 2 - circle.radius)
    circle.setTop(this._dragSteering.height / 2 - circle.radius)
    this._dragSteering.add(circle)

    const wheelHandleRadius = 20
    this._wheelCenter = [
      this._dragSteering.width / 2 - wheelHandleRadius,
      this._dragSteering.height / 2 - wheelHandleRadius
    ]

    // 设置控制柄
    this._wheelHandle = new fabric.Circle({
      left: this._wheelCenter[0],
      top: this._wheelCenter[1],
      radius: wheelHandleRadius,
      fill: '#f55',
      opacity: 0.5
    })
    this._wheelHandle.hasControls = this._wheelHandle.hasBorders = false
    this._dragSteering.add(this._wheelHandle)

    // 订阅事件
    this._dragSteering
      .on('mouse:down', (options: { e: MouseEvent, target }) => {  // 画布点击
        if (options.target == this._wheelHandle) {  // 如果点击的是控制柄
          this._wheelHandle.setFill('green')
          this._wheelHandlePress = true
        }

      })
      .on('mouse:up', (options: { e: MouseEvent, target }) => {  // 画布点击后松开
        this._wheelHandlePress = false  // 不管松开的位置位不位于控制柄上都应该设置为false
        if (options.target == this._wheelHandle) {  // 如果点击的是控制柄

          this._wheelHandle.setFill('#f55')
          // 让控制柄回到中心
          this._wheelHandle.animate({ 'left': this._wheelCenter[0], 'top': this._wheelCenter[1] }, {
            onChange: () => {
              this._dragSteering.renderAll()
            },
            onComplete: () => {
              this._flag = [0, 0]
            },
            duration: 200,
            easing: fabric.util.ease.easeInSine

          })

        }
      })
      .on('object:moving', (options: { e: MouseEvent, target }) => {  // 画布有物体移动
        if (options.target == this._wheelHandle) {  // 如果移动的是控制柄
          // 计算_wheelHandle的圆心的极坐标
          let wheelHandlePolarPoint = this.layerPoint2PolarPoint(
            this._wheelHandle.getLeft() + this._wheelHandle.radius,
            this._wheelHandle.getTop() + this._wheelHandle.radius
          )

          if (wheelHandlePolarPoint[1] > this._wheelRadius) {
            // 超出圆盘，将其置于其圆心与原点所构成的直线和圆的交点处的内侧
            let targetLayerPoint = this.polarPoint2LayerPoint(wheelHandlePolarPoint[0], this._wheelRadius)

            this._wheelHandle.setLeft(targetLayerPoint[0] - this._wheelHandle.radius)
            this._wheelHandle.setTop(targetLayerPoint[1] - this._wheelHandle.radius)

          }

          // 更新极坐标信息
          let dragPosition = this.layerPoint2PolarPoint(
            this._wheelHandle.getLeft() + this._wheelHandle.radius,
            this._wheelHandle.getTop() + this._wheelHandle.radius)
          this._flag = [dragPosition[0], parseFloat((dragPosition[1] / this._wheelRadius).toFixed(2))]
        }
      })

  }

  /**
   * 使用MotorService根据_flag不断更新电机速度，如果速度值更改，则使用WebSocketService发送命令给后台让后台控制电机
   */
  controlMotors() {
    this._mtService.changeSpeed(this.CONTROLMODE, this._flag)
  }

  /**
   * 将画布上的点从layer坐标转换为以画布中新为原点，x轴为极轴的极坐标（角度显示）
   * @param layerX
   * @param layerY
   * @returns {number[]}
   */
  layerPoint2PolarPoint(layerX: number, layerY: number): [number, number] {

    let theta, length, x, y
    x = layerX - this._dragSteering.width / 2
    y = this._dragSteering.height / 2 - layerY
    length = Math.sqrt(x * x + y * y)
    if (length == 0)
      theta = 0
    else
      theta = Math.asin(Math.abs(y) / length) * 180 / Math.PI

    if (x == 0) {
      if (y > 0)
        theta = 90
      else
        theta = 270
    } else if (y == 0) {
      if (x > 0)
        theta = 0
      else
        theta = 180
    }
    else if (x < 0 && y > 0)
      theta = 180 - theta
    else if (x < 0 && y < 0)
      theta = 180 + theta
    else if (x > 0 && y < 0)
      theta = 360 - theta
    return [theta.toFixed(2), length.toFixed(2)]
  }

  /**
   * 极坐标转换为layer坐标
   * @param theta
   * @param length
   * @returns {number[]}
   */
  polarPoint2LayerPoint(theta: number, length: number): [number, number] {
    let layerX, layerY, x, y
    x = Math.cos(theta * Math.PI / 180) * length  // 开发过程中忘了角度与弧度的转换，浪费了很多时间，菜的抠脚
    y = Math.sin(theta * Math.PI / 180) * length

    layerX = this._dragSteering.width / 2 + x
    layerY = this._dragSteering.height / 2 - y

    return [layerX, layerY]
  }

  /**
   * 返回拖拽的位置字符串
   * @returns {string}
   */
  get dragPosition(): string {
    return `stroke:${this._wheelHandlePress}  theta:${this._flag[0]}  p:${this._flag[1]}`
  }

  get motorLeftSpeed() {
    return this._mtService.motorLeftSpeed
  }

  get motorRightSpeed() {
    return this._mtService.motorRightSpeed
  }

  get motorInfo() {
    return `${MotorState[this._mtService.motorState]}  L:${this._mtService.motorLeftSpeed}  R:${this._mtService.motorRightSpeed}`
  }

  get titleShow() {
    // return `L: ${this.motorLeftSpeed}; R: ${this.motorRightSpeed}; ${MotorState[this._mtService.motorState]}`
    return `L: ${this.motorLeftSpeed}; R: ${this.motorRightSpeed}`
  }
}