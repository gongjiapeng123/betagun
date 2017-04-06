/**
 * Created by Alvin.liu on 2016/11/26.
 * 方向盘控制
 */


import { 
  Component, 
  OnInit,
  OnDestroy,
  OnChanges,
  ViewChild,
} from '@angular/core'
import {
  WebSocketService,
  MotorService
} from  '../../../service-share/services'
import { MotorState } from '../../../service-share/services/motor'
// import {Subject} from 'rxjs/Subject'


@Component({
  selector: 'steering',
  styles: [require('./steering.component.scss')],
  template: require('./steering.component.html')

})
export class SteeringComponent implements OnInit, OnDestroy, OnChanges {
  @ViewChild('steeringWheel') steeringWheel
  @ViewChild('steeringDrag') steeringDrag
  isKey: boolean = true  // 是否为按键模式

  constructor (private _wsService: WebSocketService,
               private _mtService: MotorService) {

  }

  ngOnInit () {
    

  }

  ngOnDestroy () {
    
  }

  ngOnChanges () {

  }

  onCircleClick () {
    this._mtService.runCircle()
  }

  onResetClick () {
    this._mtService.stopCircle()
  }

  get titleShow () {
    return `L: ${this._mtService.motorLeftSpeed}; R: ${this._mtService.motorRightSpeed}`
  }
}
