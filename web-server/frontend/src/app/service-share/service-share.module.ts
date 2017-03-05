/**
 * Created by alvin.liu on 2016/11/20.
 */

import { NgModule }      from '@angular/core';

import {
  MotorService,
  WebSocketService,
  CsvService,
} from './services'

const services = [
  MotorService,
  WebSocketService,
  CsvService,
]



@NgModule({
  providers: [...services]
})
export class ServiceShareModule {
}

