/**
 * Created by Alvin.liu on 2016/12/18.
 * 数据模型
 */

export interface JY901Data {
  ax: number
  ay: number
  az: number  // 加速度
  wx: number
  wy: number
  wz: number  // 角速度
  pitch: number
  roll: number
  yaw: number  // 角度
  temperature: number  // 芯片温度
}

export interface ArdiunoData {
  HUMI: number  // 湿度
  TEMP: number  // 温度
  warning_down_left_front: boolean
  warning_down_right_front: boolean
  warning_down_right_back: boolean
  warning_down_left_back: boolean  // 向下的红外，true说明远离地面
  warning_front_left_front: boolean
  warning_front_right_front: boolean
  warning_front_right_back: boolean
  warning_front_left_back: boolean  // 向前的红外，true说明前方有障碍
  dist0: number
  dist1: number
  dist2: number
  dist3: number  // 超声波
  infraredShow: any
}
