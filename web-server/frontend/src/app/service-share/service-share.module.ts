/**
 * Created by alvin.liu on 2016/11/20.
 */

import { NgModule }      from '@angular/core';

import {
  MotorService,
  WebSocketService,
} from './services'

const services = [
  MotorService,
  WebSocketService,
]



@NgModule({
  providers: [...services]
})
export class ServiceShareModule {
}

