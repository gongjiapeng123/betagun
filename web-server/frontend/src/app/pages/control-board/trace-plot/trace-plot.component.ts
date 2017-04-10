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
 * 该场景z向北，x向西，y向上，小车初始指向北，对于REP103，小车的base_link
 * 的x指向前，y指向左，Z指向上方，故对于 场景 => ROS 变换:
 * z => x; x => y; y => z
 */

import {
  Component,
  OnInit,
  OnDestroy,
  OnChanges,
  ElementRef,
  Input,
} from '@angular/core'
import {
  CsvService,
  WebSocketService,
  JY901Data,
  ArdiunoData,
  OdomData,
} from  '../../../service-share/services'
import { Subscription }   from 'rxjs/Subscription'
import * as THREE from 'three'
import { SelectItem } from 'primeng/primeng'

@Component({
  selector: 'trace-plot',
  styles: [require('./trace-plot.component.scss')],
  template: require('./trace-plot.component.html')
})
export class TracePlotComponent implements OnInit, OnDestroy, OnChanges {
  @Input() loginUser
  MAX_POINTS = 2000  // 轨迹最大点数
  private _odomsSelected: string[] = ['to']  // 选择的轨迹
  private _odoms: SelectItem[] = [  // 选择的轨迹
    {
      label: 'eo',  // 本文里程计
      value: 'eo'
    },
    {  
      label: 'wo',  // 轮式里程计
      value: 'wo'
    }, 
    {
      label: 'io',  // 惯导里程计
      value: 'io'
    }, 
    {
      label: 'to',  // 传统里程计（wo + io）
      value: 'to'
    }, 
    {
      label: 'vo',  // 双目里程计
      value: 'vo'
    }
  ]
  private _status: string = '2D'  // 位姿2d模式
  
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
  
  // 轨迹

  // eo
  private _eoTraceGeometry = new THREE.BufferGeometry()
  private _eoPositions = new Float32Array(this.MAX_POINTS * 3)  // (x, y, z)
  private _eoMaterial = new THREE.LineBasicMaterial({  // 线的材质
    color: 0xff0088,
    linewidth: 2
  })
  private _eoTraceLine = new THREE.Line(this._eoTraceGeometry, this._eoMaterial)

  // wo
  private _woTraceGeometry = new THREE.BufferGeometry()
  private _woPositions = new Float32Array(this.MAX_POINTS * 3)
  private _woMaterial = new THREE.LineBasicMaterial({
    color: 0xff8800,
    linewidth: 2
  })
  private _woTraceLine = new THREE.Line(this._woTraceGeometry, this._woMaterial)

  // io
  private _ioTraceGeometry = new THREE.BufferGeometry()
  private _ioPositions = new Float32Array(this.MAX_POINTS * 3)
  private _ioMaterial = new THREE.LineBasicMaterial({
    color: 0xff3333,
    linewidth: 2
  })
  private _ioTraceLine = new THREE.Line(this._ioTraceGeometry, this._ioMaterial)

  // to
  private _toTraceGeometry = new THREE.BufferGeometry()
  private _toPositions = new Float32Array(this.MAX_POINTS * 3)
  private _toMaterial = new THREE.LineBasicMaterial({
    color: 0x00ff00,
    linewidth: 2
  })
  private _toTraceLine = new THREE.Line(this._toTraceGeometry, this._toMaterial)

  // vo
  private _voTraceGeometry = new THREE.BufferGeometry()
  private _voPositions = new Float32Array(this.MAX_POINTS * 3)
  private _voMaterial = new THREE.LineBasicMaterial({
    color: 0x0000ff,
    linewidth: 2
  })
  private _voTraceLine = new THREE.Line(this._voTraceGeometry, this._voMaterial)

  private _traceIndex = 0
  private _drawCount = 0

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
  public eoData: OdomData = {
    vx: 0, vy: 0, vz: 0,  // 速度
    wx: 0, wy: 0, wz: 0,  // 角速度
    pitch: 0, roll: 0, yaw: 0,  // 角度
    x: 0, y: 0, z: 0,  // 位置
  }
  public woData: OdomData = this.eoData
  public ioData: OdomData = this.eoData
  public toData: OdomData = this.eoData
  public voData: OdomData = this.eoData
  odomData: OdomData = this.eoData

  private _jy901Subscription: Subscription
  private _arduinoSubscription: Subscription
  private _eoSubscription: Subscription
  private _woSubscription: Subscription
  private _ioSubscription: Subscription
  private _toSubscription: Subscription
  private _voSubscription: Subscription

  constructor (private _wsService: WebSocketService,
               private _csvService: CsvService,
               private _elementRef: ElementRef) {

  }

  ngOnInit () {
    this._initThree()

    this._jy901Subscription = this._wsService.jy901$
      .subscribe(jy901Data => {
          this.jy901Data = jy901Data
          
          // const a = {
          //   ax: jy901Data.ay - 0.2,
          //   ay: -jy901Data.ax - 0.9,
          //   az: jy901Data.az + 0.2,
          // }
          // if (a.ax > 1) {
          //   console.log(a)
          // }     
        }
      )

    this._arduinoSubscription = this._wsService.arduino$
      .subscribe(arduinoData => {
          this.arduinoData = arduinoData
          this._changeInfraredColor()

        }
      )

    this._eoSubscription = this._wsService.eo$
      .subscribe(eoData => {
        this.eoData = eoData
        // this.odomData = eoData
        // this._changePose()
      })
    this._woSubscription = this._wsService.wo$
      .subscribe(woData => {
        this.woData = woData

      })
    this._ioSubscription = this._wsService.io$
      .subscribe(ioData => {
        this.ioData = ioData
        // this.odomData = ioData
      })
    this._toSubscription = this._wsService.to$
      .sampleTime(200)
      .subscribe(toData => {
        this.toData = toData
        this.odomData = toData
        this._changePose()
      })
    this._voSubscription = this._wsService.vo$
      .subscribe(voData => {
        this.voData = voData

      })

    this._animation()
  }

  ngOnDestroy () {  // 取消订阅并停止动画
    this._jy901Subscription.unsubscribe()
    this._arduinoSubscription.unsubscribe()
    this._eoSubscription.unsubscribe()
    this._woSubscription.unsubscribe()
    this._ioSubscription.unsubscribe()
    this._toSubscription.unsubscribe()
    this._eoSubscription.unsubscribe()
    this._stopAnimation()
  }

  ngOnChanges () {

  }

  /**
   * 根据选择添加或取消wo、vo轨迹绘制
   * @param event
   */
  onOdomChanged (event) {
    const selectEo = this._odomsSelected.indexOf('eo') > -1
    const selectWo = this._odomsSelected.indexOf('wo') > -1
    const selectIo = this._odomsSelected.indexOf('io') > -1
    const selectTo = this._odomsSelected.indexOf('to') > -1
    const selectVo = this._odomsSelected.indexOf('vo') > -1
    if (selectEo) {
      this._scene.add(this._eoTraceLine)
    } else {
      this._scene.remove(this._eoTraceLine)
    }
    if (selectWo) {
      this._scene.add(this._woTraceLine)
    } else {
      this._scene.remove(this._woTraceLine)
    }

    if (selectIo) {
      this._scene.add(this._ioTraceLine)
    } else {
      this._scene.remove(this._ioTraceLine)
    }

    if (selectTo) {
      this._scene.add(this._toTraceLine)
    } else {
      this._scene.remove(this._toTraceLine)
    }

    if (selectVo) {
      this._scene.add(this._voTraceLine)
    } else {
      this._scene.remove(this._voTraceLine)
    }
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
    this._camera.position.set(0, 750, -1000)
    // this._camera.position.set(0, 500, -500)
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
    // 场景单位：cm
    this._grid = new THREE.GridHelper(1000, 20, 0xFFFFFF, 0x808080)
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
    // 自制小车模型
    loader.load('assets/3d/car-model.json', (obj) => {
      // console.log(obj)
      this._carMesh = obj
      this._carMesh.scale.set(15, 15, 15)  // 改变模型的大小

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

      this._configTraceLine()

    })

  }

  /**
   * 配置轨迹直线
   * @private
   */
  private _configTraceLine () {
    // eo
    this._eoTraceGeometry.addAttribute(
      'position',
      new THREE.BufferAttribute(this._eoPositions, 3)
    )
    this._eoTraceGeometry.setDrawRange(0, this._drawCount)
    this._eoTraceLine.geometry.attributes.position.needsUpdate = true
    this._scene.add(this._eoTraceLine)
    // wo
    this._woTraceGeometry.addAttribute(
      'position',
      new THREE.BufferAttribute(this._woPositions, 3)
    )
    this._woTraceGeometry.setDrawRange(0, this._drawCount)
    if (this._odomsSelected.indexOf('wo') > -1) {
      this._woTraceLine.geometry.attributes.position.needsUpdate = true
      this._scene.add(this._woTraceLine)
    }
    // io
    this._ioTraceGeometry.addAttribute(
      'position',
      new THREE.BufferAttribute(this._ioPositions, 3)
    )
    this._ioTraceGeometry.setDrawRange(0, this._drawCount)
    if (this._odomsSelected.indexOf('io') > -1) {
      this._ioTraceLine.geometry.attributes.position.needsUpdate = true
      this._scene.add(this._ioTraceLine)
    }
    // to
    this._toTraceGeometry.addAttribute(
      'position',
      new THREE.BufferAttribute(this._toPositions, 3)
    )
    this._toTraceGeometry.setDrawRange(0, this._drawCount)
    if (this._odomsSelected.indexOf('to') > -1) {
      this._toTraceLine.geometry.attributes.position.needsUpdate = true
      this._scene.add(this._toTraceLine)
    }
    // vo
    this._voTraceGeometry.addAttribute(
      'position',
      new THREE.BufferAttribute(this._voPositions, 3)
    )
    this._voTraceGeometry.setDrawRange(0, this._drawCount)
    if (this._odomsSelected.indexOf('vo') > -1) {
      this._voTraceLine.geometry.attributes.position.needsUpdate = true
      this._scene.add(this._voTraceLine)
    }
    
  }

  /**
   * ros位姿 => three
   * @param odomData
   * @returns three position {x, y, z}
   * @private
   */
  private _poseConvert (odomData: OdomData) {
    // 该场景z向北，x向西，y向上，小车初始指向北，对于REP103，小车的base_link
    // 的x指向前，y指向左，Z指向上方，故对于 场景 => ROS 变换:
    // z => x; x => y; y => z
    // 此处单位是cm
    const position = {
      x: 0,
      y: 0,
      z: 0
    }

    position.x = odomData.y * 100
    position.y = odomData.z * 100
    position.z = odomData.x * 100

    return position
  }

  /**
   * 位姿变化
   * @private
   */
  private _changePose () {
    if (this._carMesh) {  // 确保车模型加载完成
      // 改变模型的旋转角，注意要保证初始车头指向three坐标系的Z轴，
      // 车左侧指向X轴，车上侧指向Y轴 (x:0 y:Math.PI z:Math.PI)

      // ROS、jy901的轴对应three坐标轴，方向相同，传来的数据是角度制
      if (this._status === '3D') {
        this._carMesh.rotation.x = this.odomData.pitch * Math.PI / 180
        this._carMesh.rotation.z = this.odomData.roll * Math.PI / 180
      }
      this._carMesh.rotation.y = this.odomData.yaw * Math.PI / 180
 
      const position = this._poseConvert(this.odomData)
      this._carMesh.position.x = position.x
      this._carMesh.position.z = position.z
      if (this._status === '3D') {
        this._carMesh.position.y = position.y
      }

      this._drawTrace()
    }
  }

  private _changeInfraredColor () {
    if (this._carMesh) {  // 确保车模型加载完成
      this._downLeftFrontMesh.material.emissive = this.arduinoData.warning_down_left_front
        ? {r: 1, g: 0, b: 0}
        : {r: 0, g: 1, b: 0}
      this._downRightFrontMesh.material.emissive = this.arduinoData.warning_down_right_front
        ? {r: 1, g: 0, b: 0}
        : {r: 0, g: 1, b: 0}
      this._downRightBackMesh.material.emissive = this.arduinoData.warning_down_right_back
        ? {r: 1, g: 0, b: 0}
        : {r: 0, g: 1, b: 0}
      this._downLeftBacktMesh.material.emissive = this.arduinoData.warning_down_left_back
        ? {r: 1, g: 0, b: 0}
        : {r: 0, g: 1, b: 0}
      this._frontLeftFrontMesh.material.emissive = this.arduinoData.warning_front_left_front
        ? {r: 1, g: 0, b: 0}
        : {r: 0, g: 1, b: 0}
      this._frontRightFrontMesh.material.emissive = this.arduinoData.warning_front_right_front
        ? {r: 1, g: 0, b: 0}
        : {r: 0, g: 1, b: 0}
      this._frontRightBackMesh.material.emissive = this.arduinoData.warning_front_right_back
        ? {r: 1, g: 0, b: 0}
        : {r: 0, g: 1, b: 0}
      this._frontLeftBacktMesh.material.emissive = this.arduinoData.warning_front_left_back
        ? {r: 1, g: 0, b: 0}
        : {r: 0, g: 1, b: 0}
    }
  }

  /**
   * 绘制轨迹
   * @private
   */
  private _drawTrace () {
    if (this._eoTraceLine) {
      // 判断是否有位移  // cm
      const positions = this._toPositions
      let lastPointX = 0
      let lastPointY = 0
      let lastPointZ = 0
      if (this._traceIndex !== 0) {
        lastPointX = positions[this._traceIndex - 3]
        lastPointY = positions[this._traceIndex - 2]
        lastPointZ = positions[this._traceIndex - 1]
      }

      let dist2 = 0  // cm^2

      const eoPosition = this._poseConvert(this.eoData)
      const woPosition = this._poseConvert(this.woData)
      const ioPosition = this._poseConvert(this.ioData)
      const toPosition = this._poseConvert(this.toData)
      const voPosition = this._poseConvert(this.voData)
      const position = eoPosition

      if (this._status === '3D') {
        dist2 = Math.pow(lastPointX -position.x, 2)
          + Math.pow(lastPointY - position.y, 2)
          + Math.pow(lastPointZ - position.z, 2)
      } else {
        dist2 = Math.pow(lastPointX - position.x, 2)
          + Math.pow(lastPointZ - position.z, 2)
      }
      // console.log(dist2, this._traceIndex, this._drawCount)
      // 位移太小，不存入buf (5cm)
      if (dist2 < 25) {
        return
      }

      this._drawCount = (this._drawCount + 1) % this.MAX_POINTS

      this._eoTraceLine.geometry.setDrawRange(0, this._drawCount)
      this._woTraceLine.geometry.setDrawRange(0, this._drawCount)
      this._ioTraceLine.geometry.setDrawRange(0, this._drawCount)
      this._toTraceLine.geometry.setDrawRange(0, this._drawCount)
      this._voTraceLine.geometry.setDrawRange(0, this._drawCount)

      this._eoPositions[this._traceIndex] = eoPosition.x
      this._woPositions[this._traceIndex] = woPosition.x
      this._ioPositions[this._traceIndex] = ioPosition.x
      this._toPositions[this._traceIndex] = toPosition.x
      this._voPositions[this._traceIndex] = voPosition.x
      this._traceIndex++

      if (this._status === '3D') {
        this._eoPositions[this._traceIndex] = eoPosition.y
        this._woPositions[this._traceIndex] = woPosition.y
        this._ioPositions[this._traceIndex] = ioPosition.y
        this._toPositions[this._traceIndex] = toPosition.y
        this._voPositions[this._traceIndex] = voPosition.y
        this._traceIndex++
      } else {
        this._eoPositions[this._traceIndex] = 0
        this._woPositions[this._traceIndex] = 0
        this._ioPositions[this._traceIndex] = 0
        this._toPositions[this._traceIndex] = 0
        this._voPositions[this._traceIndex] = 0
        this._traceIndex++
      }
      this._eoPositions[this._traceIndex] = eoPosition.z
      this._woPositions[this._traceIndex] = woPosition.z
      this._ioPositions[this._traceIndex] = ioPosition.z
      this._toPositions[this._traceIndex] = toPosition.z
      this._voPositions[this._traceIndex] = voPosition.z
      this._traceIndex++

      // 绘制结果轨迹，根据选择是否绘制wo、vo
      this._eoTraceLine.geometry.attributes.position.needsUpdate = this._odomsSelected.indexOf('eo') > -1
      this._woTraceLine.geometry.attributes.position.needsUpdate = this._odomsSelected.indexOf('wo') > -1
      this._ioTraceLine.geometry.attributes.position.needsUpdate = this._odomsSelected.indexOf('io') > -1
      this._toTraceLine.geometry.attributes.position.needsUpdate = this._odomsSelected.indexOf('to') > -1
      this._voTraceLine.geometry.attributes.position.needsUpdate = this._odomsSelected.indexOf('vo') > -1

      if (this._traceIndex >= this.MAX_POINTS * 3) {
        this._traceIndex = 0
      }
    }
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

  /**
   * 导出轨迹数据
   */
  onExportTraceClick (event) {
    this._csvService.exportToCsv([
      this._eoPositions,
      this._woPositions,
      this._ioPositions,
      this._toPositions,
      this._voPositions,
    ])
  }

  /**
   * 重置轨迹数据
   */
  onResetTraceClick (event) {
    if (this.loginUser === 'admin') {
      this._wsService.restartPosition()
    }

    for (let i = 0; i < this._eoPositions.length; i++) {
      this._eoPositions[i] = 0
    }

    for (let i = 0; i < this._woPositions.length; i++) {
      this._woPositions[i] = 0
    }

    for (let i = 0; i < this._ioPositions.length; i++) {
      this._ioPositions[i] = 0
    }

    for (let i = 0; i < this._toPositions.length; i++) {
      this._toPositions[i] = 0
    }

    for (let i = 0; i < this._voPositions.length; i++) {
      this._voPositions[i] = 0
    }
    this._drawCount = 0
    this._traceIndex = 0

    this._eoTraceGeometry.setDrawRange(0, this._drawCount)
    this._woTraceGeometry.setDrawRange(0, this._drawCount)
    this._ioTraceGeometry.setDrawRange(0, this._drawCount)
    this._toTraceGeometry.setDrawRange(0, this._drawCount)
    this._voTraceGeometry.setDrawRange(0, this._drawCount)

    this._eoTraceLine.geometry.attributes.position.needsUpdate = this._odomsSelected.indexOf('eo') > -1
    this._woTraceLine.geometry.attributes.position.needsUpdate = this._odomsSelected.indexOf('wo') > -1
    this._ioTraceLine.geometry.attributes.position.needsUpdate = this._odomsSelected.indexOf('io') > -1
    this._toTraceLine.geometry.attributes.position.needsUpdate = this._odomsSelected.indexOf('to') > -1
    this._voTraceLine.geometry.attributes.position.needsUpdate = this._odomsSelected.indexOf('vo') > -1
  }

}
