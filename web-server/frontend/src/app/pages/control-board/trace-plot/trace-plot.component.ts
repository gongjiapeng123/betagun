/**
 * Created by Alvin.liu on 2016/6/12.
 * 显示位姿图
 */

import {
  Component,
  OnInit,
  OnDestroy,
  ElementRef,
} from '@angular/core'
import { WebSocketService } from  '../../../service-share/services'
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

  public jy901Data = {
    ax: 0, ay: 0, az: 0,  // 加速度
    wx: 0, wy: 0, wz: 0,  // 角速度
    pitch: 0, roll: 0, yaw: 0,  // 角度
    temperature: 0,  // 温度
  }
  public arduinoData = {
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
    this._camera.position.set(0, 100, -100)
    this._camera.up.set(0, 0, 1)
    this._camera.lookAt(new THREE.Vector3(0, 0, 0))
    this._scene.add(this._camera)

    // 定义相机控制器

    // 定义光线，位置不同，方向光作用于物体的面也不同，看到的物体各个面的颜色也不一样
    this._light0 = new THREE.DirectionalLight(0xFF0000, 1)
    this._light0.position.set(1, 1, 1)
    this._scene.add(this._light0)
    this._light1 = new THREE.DirectionalLight(0x11FF22, 1)
    this._light1.position.set(-1, 1, 1)
    this._scene.add(this._light1)
    this._light2 = new THREE.DirectionalLight(0x0000FF, 1)
    this._light2.position.set(-1, -1, 1)
    this._scene.add(this._light2)
    this._light3 = new THREE.DirectionalLight(0x880066, 1)
    this._light3.position.set(1, 1, -1)
    this._scene.add(this._light3)

    // 定义网格，网格的边长是1000，每个小网格的边长是50
    this._grid = new THREE.GridHelper(1000, 50);
    this._grid.setColors(0x0000ff, 0x808080);
    this._scene.add(this._grid)

    // 定义车体
    // this._carMesh = new THREE.Mesh(new THREE.CubeGeometry(30, 30, 50, 4, 4, 4),
    //   new THREE.MeshLambertMaterial({  // Lambert材质会受环境光的影响，呈现环境光的颜色，与材质本身颜色关系不大
    //     color: 0xFFFFFF,
    //   })
    // )
    // this._carMesh.position.set(0, 0, 0)
    // this._scene.add(this._carMesh)

    // let loader = new THREE.JSONLoader()
    // loader.load( 'public/3D_models/mclaren-mp4-12c-vray.json', function ( geometry, materials ) {
    //   this._carMesh = new THREE.Mesh( geometry, new THREE.MeshFaceMaterial( materials ) );
    //   this._carMesh.position.set(0, 0, 0)
    //   this._scene.add(this._carMesh)
    //
    //   console.log(this._carMesh)
    //
    // });

    const loader = new THREE.ObjectLoader()
    loader.load('assets/3d/mclaren-mp4-12c-vray.json', (obj) => {
      // console.log(obj)
      this._carMesh = obj.children[1]
      this._carMesh.scale.set(0.005, 0.005, 0.005)  // 改变模型的大小

      this._changePose()

      this._carMesh.position.set(0, 0, 0)
      this._scene.add(this._carMesh)

      // console.log(this._carMesh)
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

      // jy901的x轴与three的相反，所以俯仰角要取负号
      this._carMesh.rotation.x = -this.jy901Data.pitch * Math.PI / 180
      // 加上初始化时的角度
      this._carMesh.rotation.y = Math.PI + this.jy901Data.yaw * Math.PI / 180
      // jy901的x轴与three的相反，所以侧滚角要取负号
      this._carMesh.rotation.z = Math.PI - this.jy901Data.roll * Math.PI / 180
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
    const p = this.jy901Data.pitch.toFixed(2)
    const r = this.jy901Data.roll.toFixed(2)
    const y = this.jy901Data.yaw.toFixed(2)
    return `p: ${p}; r: ${r}; y: ${y}`

  }

  ngOnInit () {
    this._initThree()

    this._jy901Subscription = this._wsService.jy901$
      .subscribe((jy901Data) => {
          this.jy901Data = jy901Data
          this._changePose()

        }
      )

    this._arduinoSubscription = this._wsService.arduino$
      .subscribe((arduinoData) => {
          this.arduinoData = arduinoData

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
