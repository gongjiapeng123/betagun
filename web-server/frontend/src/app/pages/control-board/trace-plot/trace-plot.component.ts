/**
 * Created by Alvin.liu on 2016/6/12.
 * 显示位姿图
 *
 * jy901的坐标系：
 * 右手，相对于传感器
 * X指向右侧，Y指向前方，Z指向上方
 * 绕X旋转为pitch，Y为roll，Z为yaw
 *
 * three的坐标系：
 * 右手，相对于屏幕
 * X指向右侧，Y指向上方，Z指向频幕外
 * 此处设定摄像头在高度100处沿着Z轴观看场景，所以设定
 * 场景向里指向北方，向外指向南方，故实际上：
 * X指向左侧（西），Y指向上方，Z指向频幕里（北）
 *
 */

import {
  Component,
  OnInit,
  OnDestroy,
  ElementRef,
} from '@angular/core'
import {
  WebSocketService,
  JY901Data,
  ArdiunoData,
} from  '../../../service-share/services/websocket'
import { Subscription }   from 'rxjs/Subscription'
import * as THREE from 'three'

@Component({
  selector: 'trace-plot',
  styles: [require('./trace-plot.component.scss')],
  template: require('./trace-plot.component.html')
})
export class TracePlotComponent implements OnInit, OnDestroy {

  private _canvasEl: HTMLCanvasElement
  private _id  // requestAnimationFrame id
  // three.js 对象
  private _renderer
  private _scene
  private _camera
  private _controls
  private _light0
  private _light1
  private _light2
  private _light3
  private _grid
  private _carMesh

  private _downLeftFrontMesh
  private _downRightFrontMesh
  private _downLeftBacktMesh
  private _downRightBackMesh
  private _frontLeftFrontMesh
  private _frontRightFrontMesh
  private _frontLeftBacktMesh
  private _frontRightBackMesh

  public jy901Data: JY901Data = {
    ax: 0, ay: 0, az: 0,  // 加速度
    wx: 0, wy: 0, wz: 0,  // 角速度
    pitch: 0, roll: 0, yaw: 0,  // 角度
    temperature: 0,  // 温度
  }
  public arduinoData: ArdiunoData = {
    HUMI: 0, TEMP: 0,  // 温湿度
    warning_down_left_front: false,
    warning_down_right_front: false,
    warning_down_right_back: false,
    warning_down_left_back: false,  // 向下的红外，true说明远离地面
    warning_front_left_front: false,
    warning_front_right_front: false,
    warning_front_right_back: false,
    warning_front_left_back: false,  // 向前的红外，true说明前方有障碍
    dist0: 0, dist1: 0, dist2: 0, dist3: 0,  // 超声波
    infraredShow: {
      down: '0000',
      front: '0000',
    }
  }

  private _jy901Subscription: Subscription
  private _arduinoSubscription: Subscription

  constructor (private _wsService: WebSocketService,
               private _elementRef: ElementRef) {

  }

  /**
   * 初始化webgl
   * @private
   */
  private _initThree () {
    // 定义渲染器
    this._canvasEl = <HTMLCanvasElement>this
      ._elementRef
      .nativeElement
      .querySelector('#canvas')
    this._renderer = new THREE.WebGLRenderer({
      canvas: this._canvasEl,
      antialias: true,
    })
    this._renderer.setClearColor(0x666666, 1.0)  // 底色

    // 定义场景
    this._scene = new THREE.Scene()

    // 定义相机
    // this._camera = new THREE.OrthographicCamera(
    //   this._canvasEl.width / -2, this._canvasEl.width / 2,
    //   this._canvasEl.height / -2, this._canvasEl.height / 2,
    //   20, 1000
    // )
    this._camera = new THREE.PerspectiveCamera(
      45, this._canvasEl.width / this._canvasEl.height,
      1, 10000
    )

    // 在高度100处沿着Z轴观看场景，此时Z轴向频幕里（北）Y轴向上，X轴向左（西）
    this._camera.position.set(0, 300, -300)
    this._camera.up.set(0, 0, 1)
    this._camera.lookAt(new THREE.Vector3(0, 0, 0))
    this._scene.add(this._camera)

    // 定义相机控制器

    // 定义光线，位置不同，方向光作用于物体的面也不同，看到的物体各个面的颜色也不一样
    this._light0 = new THREE.DirectionalLight(0xFFFFFF, 1)
    this._light0.position.set(1, 1, 1)
    this._scene.add(this._light0)
    // this._light1 = new THREE.DirectionalLight(0x11FF22, 1)
    // this._light1.position.set(-1, 1, 1)
    // this._scene.add(this._light1)
    // this._light2 = new THREE.DirectionalLight(0x0000FF, 1)
    // this._light2.position.set(-1, -1, 1)
    // this._scene.add(this._light2)
    // this._light3 = new THREE.DirectionalLight(0x880066, 1)
    // this._light3.position.set(1, 1, -1)
    // this._scene.add(this._light3)

    // 定义网格，网格的边长是1000，每个小网格的边长是50
    this._grid = new THREE.GridHelper(1000, 50, 0xFFFFFF, 0x808080)
    this._scene.add(this._grid)

    const loader = new THREE.ObjectLoader()
    // 靓车
    // loader.load('assets/3d/mclaren-mp4-12c-vray.json', (obj) => {
    //   this._carMesh = obj.children[1]
    //   this._carMesh.scale.set(0.005, 0.005, 0.005)  // 改变模型的大小
    //
    //   this._changePose()
    //
    //   this._carMesh.position.set(0, 0, 0)
    //   this._scene.add(this._carMesh)
    // })
    loader.load('assets/3d/car-model.json', (obj) => {
      // console.log(obj)
      this._carMesh = obj
      this._carMesh.scale.set(6, 6, 6)  // 改变模型的大小

      this._changePose()
      this._carMesh.position.set(0, 0, 0)

      this._downLeftFrontMesh = this._carMesh.getObjectByName('downLeftFrontInfrared')
      this._downRightFrontMesh = this._carMesh.getObjectByName('downRightFrontInfrared')
      this._downRightBackMesh = this._carMesh.getObjectByName('downRightBackInfrared')
      this._downLeftBacktMesh = this._carMesh.getObjectByName('downLeftBackInfrared')
      this._frontLeftFrontMesh = this._carMesh.getObjectByName('frontLeftFrontInfrared')
      this._frontRightFrontMesh = this._carMesh.getObjectByName('frontRightFrontInfrared')
      this._frontRightBackMesh = this._carMesh.getObjectByName('frontRightBackInfrared')
      this._frontLeftBacktMesh = this._carMesh.getObjectByName('frontLeftBackInfrared')

      this._changeInfraredColor()

      this._scene.add(this._carMesh)
      console.log(this._scene);
    })

  }

  /**
   * 位姿变化
   * @private
   */
  private _changePose () {
    if (this._carMesh) {  // 确保车模型加载完成
      // 改变模型的旋转角，注意要保证初始车头指向three坐标系的Z轴，
      // 车左侧指向X轴，车上侧指向Y轴 (x:0 y:Math.PI z:Math.PI)

      // jy901的x轴对应three坐标轴x，方向相同
      this._carMesh.rotation.x = this.jy901Data.pitch * Math.PI / 180
      // jy901的Z轴对应three的y轴，方向相同
      this._carMesh.rotation.y = this.jy901Data.yaw * Math.PI / 180
      // jy901的y轴对应three的z轴，方向相同
      this._carMesh.rotation.z = this.jy901Data.roll * Math.PI / 180
    }
  }

  private _changeInfraredColor () {
    this._downLeftFrontMesh.material.emissive = this.arduinoData.warning_down_left_front
      ? {r: 0, g: 1, b: 0}
      : {r: 1, g: 0, b: 0}
    this._downRightFrontMesh.material.emissive = this.arduinoData.warning_down_right_front
      ? {r: 0, g: 1, b: 0}
      : {r: 1, g: 0, b: 0}
    this._downRightBackMesh.material.emissive = this.arduinoData.warning_down_right_back
      ? {r: 0, g: 1, b: 0}
      : {r: 1, g: 0, b: 0}
    this._downLeftBacktMesh.material.emissive = this.arduinoData.warning_down_left_back
      ? {r: 0, g: 1, b: 0}
      : {r: 1, g: 0, b: 0}
    this._frontLeftFrontMesh.material.emissive = this.arduinoData.warning_front_left_front
      ? {r: 0, g: 1, b: 0}
      : {r: 1, g: 0, b: 0}
    this._frontRightFrontMesh.material.emissive = this.arduinoData.warning_front_right_front
      ? {r: 0, g: 1, b: 0}
      : {r: 1, g: 0, b: 0}
    this._frontRightBackMesh.material.emissive = this.arduinoData.warning_front_right_back
      ? {r: 0, g: 1, b: 0}
      : {r: 1, g: 0, b: 0}
    this._frontLeftBacktMesh.material.emissive = this.arduinoData.warning_front_left_back
      ? {r: 0, g: 1, b: 0}
      : {r: 1, g: 0, b: 0}
  }

  /**
   * 绘制动画
   * @private
   */
  private _animation () {

    this._renderer.render(this._scene, this._camera)

    this._id = requestAnimationFrame(() => {
      this._animation()
    })
  }

  /**
   * 停止绘制动画
   * @private
   */
  private _stopAnimation () {
    if (this._id !== null) {
      cancelAnimationFrame(this._id)
      this._id = null
    }
  }

  get titleShow () {
    return '轨迹图'
  }

  ngOnInit () {
    this._initThree()

    this._jy901Subscription = this._wsService.jy901$
      .subscribe(jy901Data => {
          this.jy901Data = jy901Data
          this._changePose()

        }
      )

    this._arduinoSubscription = this._wsService.arduino$
      .subscribe(arduinoData => {
          this.arduinoData = arduinoData
          this._changeInfraredColor()

        }
      )

    this._animation()
  }

  ngOnDestroy () {  // 取消订阅jy901Observable 并 停止动画
    this._jy901Subscription.unsubscribe()
    this._arduinoSubscription.unsubscribe()
    this._stopAnimation()
  }

}
