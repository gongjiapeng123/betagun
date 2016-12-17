/**
 * Created by alvin.liu on 2016/11/26.
 * 显示小车传来的图像
 */

import {
  Component,
  OnInit,
  OnDestroy,
  ElementRef,
} from '@angular/core'
import { WebSocketService } from  '../../../service-share/services'
// import {Subject} from 'rxjs/Subject'
import { Subscription }   from 'rxjs/Subscription'


@Component({
  selector: 'beta-image',
  styles: [require('./beta-image.component.scss')],
  template: require('./beta-image.component.html')
})
export class BetaImageComponent implements OnInit, OnDestroy {
  fps: number = 0  // 视频帧数
  loginUser: string
  private _loginUserSubscription: Subscription
  private _imageSubscription: Subscription  // 对loginObservable的订阅
  private _canvasContext
  private _imgEl: HTMLImageElement
  private _imgCount: number = 0  // 接收的图片数
  private _imgDrawn: boolean  // 当前图片绘制是否完成
  private _fpsTimerID  // 更新FPS的定时器id，每秒计算一次FPS
  constructor (private _wsService: WebSocketService,
               private _elementRef: ElementRef) {

  }

  private _drawImage (imgDataArray: ArrayBuffer) {
    this._imgCount++

    // 之前的已绘制成功
    if (this._imgDrawn) {
      // 使用类型数组和Blob对象创建一个对象URL
      let imageBlob = new Blob(
        [imgDataArray],
        {type: "image/jpeg"}
      )  // warning: UC浏览器在这里有问题

      this._imgEl.src = URL.createObjectURL(imageBlob)
      this._imgDrawn = false
    }

  }

  ngOnInit () {

    this._canvasContext = (<HTMLCanvasElement>this
      ._elementRef
      .nativeElement
      .querySelector('#canvas'))
      .getContext('2d')

    this._imgEl = new Image()
    this._imgEl.src = './assets/img/preload.png'  // 初始加载一个背景图

    // 图片加载需要时间，在图片加载过程中，如果有图片传输过来时，执行了
    // this.img.src的更新会导致onload的重置，从而绘图卡顿，可以考虑当
    // 加载之前的图片未完成时丢弃新获取的图片
    this._imgDrawn = false
    this._imgEl.onload = () => {
      this._canvasContext.drawImage(this._imgEl, 0, 0)
      this._imgDrawn = true
    }

    this._loginUserSubscription = this._wsService.loginUser$
      .subscribe(loginUser => {
        this.loginUser = loginUser

        if (loginUser === 'admin') {
          this._wsService.imageOn()
          this._imageSubscription = this._wsService.image$.subscribe(
            (image: ArrayBuffer) => {
              this._drawImage(image)
            }
          )
          // 计算FPS
          this._fpsTimerID && clearInterval(this._fpsTimerID)
          this._fpsTimerID = setInterval(() => {
            this.fps = this._imgCount
            this._imgCount = 0
          }, 1000)

          if (loginUser === 'nobody') {  // 如果登出或未登录
            this._clean()
          }
        }
      })

  }

  ngOnDestroy () {
    this._clean()
  }

  _clean () {
    this._imageSubscription
    && this._imageSubscription.unsubscribe()
    && this._wsService.imageOff()

    clearInterval(this._fpsTimerID)
  }

  get titleShow () {
    return `双目图像 (fps: ${this.fps})`
  }

}
