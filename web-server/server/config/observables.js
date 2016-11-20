/**
 * Created by alvin.liu on 2016/6/12.
 * 可观测数据流，用于给websocket模块订阅并发送给客户端
 */

import Rx from 'rxjs/Rx'

export const image$ = new Rx.Subject()  // 发射图像流
export const jy901$ = new Rx.Subject()  // 发射jy901数据流
export const arduino$ = new Rx.Subject()  // 发射arduino的传感器信息
export const info$ = new Rx.Subject()  // 发射小车运算结果信息
