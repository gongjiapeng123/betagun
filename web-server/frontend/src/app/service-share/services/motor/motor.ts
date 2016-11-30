/**
 * Created by Alvin.liu on 2016/4/26.
 * 电机控制，左电机，右电机
 */

const MAX_SPEED = 100  // 电机速度最大值

/**
 * 双电机控制
 */
export class DualMotor {
  motorLeft: Motor
  motorRight: Motor
  preMotorState: MotorState = MotorState.Stop // 每次调用change时，在函数结束前都会赋值为curMotorState（仅作为可读性的提升）
  curMotorState: MotorState = MotorState.Stop  // 每次调用change时，在函数结束前都会进行更新

  constructor () {
    this.motorLeft = new Motor()
    this.motorRight = new Motor()
  }

  /**
   * 拖拽方式控制小车，这里点击根据拖拽位置直接设置电机速度，没有变化过程
   * @param flag 拖拽点相对于圆中心的长度归一化的极坐标 [theta, length]
   */
  public changeSpeedByDrag (flag: [number, number]) {
    this.preMotorState = this.curMotorState  // 状态更新

    let theta = flag[0], length = flag[1]
    if (length == 0) {  // 小车应静止
      this.stop()

    } else {  // 应根据角度判断
      if (theta >= 0 && theta < 90) {  // 小车右拐，左电机速度只受长度影响，右电机速度受长度和角度影响
        this.motorLeft.speed = length * MAX_SPEED
        this.motorRight.speed = this.motorLeft.speed * Math.cos((90 - theta) * Math.PI / 180)

      } else if (theta >= 90 && theta < 180) { // 小车左拐，右电机速度只受长度影响，左电机速度受长度和角度影响
        this.motorRight.speed = length * MAX_SPEED
        this.motorLeft.speed = this.motorRight.speed * Math.cos((theta - 90) * Math.PI / 180)

      } else if (theta >= 180 && theta < 270) {  // 小车倒车左拐，右电机速度只受长度影响，左电机速度受长度和角度影响
        this.motorRight.speed = -length * MAX_SPEED
        this.motorLeft.speed = this.motorRight.speed * Math.cos((270 - theta) * Math.PI / 180)

      } else if (theta >= 270 && theta < 360) {  // 小车倒车右拐，左电机速度只受长度影响，右电机速度受长度和角度影响
        this.motorLeft.speed = -length * MAX_SPEED
        this.motorRight.speed = this.motorLeft.speed * Math.cos((theta - 270) * Math.PI / 180)
      }

      this.motorLeft.speed = Math.round(this.motorLeft.speed)
      this.motorRight.speed = Math.round(this.motorRight.speed)
    }

    this.updateMotorState();

  }

  /**
   * 按键方式控制小车（此处控制模仿惯性，小车停止，回正或转向有惯性，但小车移动方向相反时则立刻静止并变向）
   * @param flag 方向按键标识,4bit二进制分别对应wsad
   */
  public changeSpeedByKey (flag: number) {
    this.preMotorState = this.curMotorState  // 状态更新
    this._changeSpeedByKey(flag)
    this.updateMotorState();
    // console.log(`${flag} ${this.motorLeft.speed} and ${this.motorRight.speed}`)
  }

  /**
   * 根据方向按键标识改变motor的速度值
   * @param flag 方向按键标识,4bit二进制分别对应wsad
   */
  private _changeSpeedByKey (flag) {
    switch (flag) {
      case 0b0000: {  // 0000 静止
        switch (this.preMotorState) { // 根据前一刻确定减速的的快慢
          case MotorState.Stop:
            this.stop();
            break;
          case MotorState.Forward:
            this.dualDec(8, 8);
            break;
          case MotorState.ForwardAndTurnRight:
            this.dualDec(8, 5);  // 转弯
            break;
          case MotorState.ForwardAndTurnLeft:
            this.dualDec(5, 8);
            break;
          case MotorState.Back:
            this.dualDec(8, 8);
            break;
          case MotorState.BackAndTurnRight:
            this.dualDec(8, 5);  // 转弯
            break;
          case MotorState.BackAndTurnLeft:
            this.dualDec(5, 8);
            break;
          case MotorState.ForwardRightCircle:
            this.dualDec(8, 5);  // 打转减速的方式与转弯相同
            break;
          case MotorState.ForwardLeftCircle:
            this.dualDec(5, 8);
            break;
          case MotorState.BackRightCircle:
            this.dualDec(8, 5);
            break;
          case MotorState.BackdLeftCircle:
            this.dualDec(5, 8);
            break;
          case MotorState.CWCircle:
            this.dualDec(8, 8);
            break;
          case MotorState.CCWCircle:
            this.dualDec(8, 8);
            break;
          default:
            break;
        }
      }
        break;
      case 0b0001: {  // 0001 右,(仅仅按左右键，不按前进和后退时，两个电机的速度应该下降，但差值应逐步增大)
        switch (this.preMotorState) {
          case MotorState.Stop:
            this.stop();  // 前一刻静止，则此刻仍静止
            break;
          case MotorState.Forward:
            this.dec(2, 5);  // 前一刻前进，则此刻前进减速，右向转弯
            break;
          case MotorState.ForwardAndTurnRight:
            this.dec(2, 5);  // 前一刻前进右转弯，则此刻前进减速，右向转弯
            break;
          case MotorState.ForwardAndTurnLeft:
            this.dec(2, 6);  // 前一刻前进左转弯，则此刻前进减速，右向转弯，右电机减速多一些，使回正效果明显
            break;
          case MotorState.Back:
            this.dec(2, 5);  // 前一刻后退，则此刻后退减速，右向转弯
            break;
          case MotorState.BackAndTurnRight:
            this.dec(2, 5);  // 前一刻后退，则此刻后退减速，右向转弯
            break;
          case MotorState.BackAndTurnLeft:
            this.dec(2, 6);  // 前一刻后退，则此刻后退减速，右向转弯,右电机减速多一些，使回正效果明显
            break;
          case MotorState.ForwardRightCircle:
            this.dec(2, 5);  // 前一刻前进右打转，则此刻前进减速，右向转弯
            break;
          case MotorState.ForwardLeftCircle:
            this.dec(2, 6);  // 前一刻前进左打转，则此刻前进减速，右向转弯,右电机减速多一些，使回正效果明显
            break;
          case MotorState.BackRightCircle:
            this.dec(2, 5);  // 前一刻后退右打转，则此刻后退减速，右向转弯
            break;
          case MotorState.BackdLeftCircle:
            this.dec(2, 6);  // 前一刻后退左打转，则此刻后退减速，右向转弯,右电机减速多一些，使回正效果明显
            break;
          case MotorState.CWCircle:
            this.dec(2, 5);  // 前一刻原地顺时针打转，则此时减速，右向转弯
            break;
          case MotorState.CCWCircle:
            this.dec(2, 5);  // 前一刻原地逆时针打转，则此时减速，右向转弯
            break;
          default:
            break;
        }
      }
        break;
      case 0b0010: {  // 0010 左
        switch (this.preMotorState) {
          case MotorState.Stop:
            this.stop();  // 前一刻静止，则此刻仍静止
            break;
          case MotorState.Forward:
            this.dec(5, 2);  // 前一刻前进，则此刻前进减速，左向转弯
            break;
          case MotorState.ForwardAndTurnRight:
            this.dec(6, 2);  // 前一刻前进右转弯，则此刻前进减速，左向转弯，左电机减速多一些，使回正效果明显
            break;
          case MotorState.ForwardAndTurnLeft:
            this.dec(5, 2);  // 前一刻前进左转弯，则此刻前进减速，左向转弯
            break;
          case MotorState.Back:
            this.dec(5, 2);  // 前一刻后退，则此刻后退减速，左向转弯
            break;
          case MotorState.BackAndTurnRight:
            this.dec(6, 2);  // 前一刻后退，则此刻后退减速，左向转弯,左电机减速多一些，使回正效果明显
            break;
          case MotorState.BackAndTurnLeft:
            this.dec(5, 2);  // 前一刻后退，则此刻后退减速，左向转弯
            break;
          case MotorState.ForwardRightCircle:
            this.dec(6, 2);  // 前一刻前进右打转，则此刻前进减速，左向转弯,左电机减速多一些，使回正效果明显
            break;
          case MotorState.ForwardLeftCircle:
            this.dec(5, 2);  // 前一刻前进左打转，则此刻前进减速，左向转弯
            break;
          case MotorState.BackRightCircle:
            this.dec(6, 2);  // 前一刻后退右打转，则此刻后退减速，右向转弯,左电机减速多一些，使回正效果明显
            break;
          case MotorState.BackdLeftCircle:
            this.dec(5, 2);  // 前一刻后退左打转，则此刻后退减速，左向转弯
            break;
          case MotorState.CWCircle:
            this.dec(5, 2);  // 前一刻原地顺时针打转，则此时减速，左向转弯
            break;
          case MotorState.CCWCircle:
            this.dec(5, 2);  // 前一刻原地逆时针打转，则此时减速，左向转弯
            break;
          default:
            break;
        }
      }
        break;
      case 0b0011: {  // 0011 左右 按键冲突，视为停止
        this._changeSpeedByKey(0b0000)
      }
        break;
      case 0b0100: {  // 0100 下
        switch (this.preMotorState) {
          case MotorState.Stop:
            this.dualAcc(3, 3, false);  // 前一刻停止，则此刻直接双轮反向加速
            break;
          case MotorState.Forward:
            this.stop();  // 前一刻前进，则此刻立刻停止后直接双轮反向加速
            break;
          case MotorState.ForwardAndTurnRight:
            this.stop();  // 前一刻前进向右，则此刻立刻停止后直接双轮反向加速
            break;
          case MotorState.ForwardAndTurnLeft:
            this.stop();  // 前一刻前进向左，则此刻立刻停止后直接双轮反向加速
            break;
          case MotorState.Back:
            this.dualAcc(3, 3, false);  // 前一刻后退，则此刻直接双轮反向加速
            break;
          case MotorState.BackAndTurnRight:
            this.dualAcc(3, 5, false);  // 前一刻后退向右转，则此刻双轮反向加速，但右电机加速应更快，需回正
            break;
          case MotorState.BackAndTurnLeft:
            this.dualAcc(5, 3, false);  // 前一刻后退向左转，则此刻双轮反向加速，但左电机加速应更快，需回正
            break;
          case MotorState.ForwardRightCircle:
            this.stop();  // 前一刻前进右打转，则此刻立刻停止后直接双轮反向加速
            break;
          case MotorState.ForwardLeftCircle:
            this.stop();  // 前一刻前进左打转，则此刻立刻停止后直接双轮反向加速
            break;
          case MotorState.BackRightCircle:
            this.motorRight.speed = 0;  // 前一刻后退右打转，左电机必定反向满速，右电机正向，则此刻立刻归零右电机
            break;
          case MotorState.BackdLeftCircle:
            this.motorLeft.speed = 0;  // 前一刻后退左打转，右电机必定反向满速，左电机正向，则此刻立刻归零左电机
            break;
          case MotorState.CWCircle:
            this.stop();  // 前一刻顺时针原地打转，则此刻立刻停止后直接双轮反向加速
            break;
          case MotorState.CCWCircle:
            this.stop();  // 前一刻逆时针原地打转，则此刻立刻停止后直接双轮反向加速
            break;
          default:
            break;
        }
      }
        break;
      case 0b0101: {  // 0101 下右
        switch (this.preMotorState) {
          case MotorState.Stop:
            this.acc(5, 3, false);  // 前一刻静止，则此刻立刻反向加速，但左电机加的较快，状态将变为后退向右
            break;
          case MotorState.Forward:
            this.stop();
            break;
          case MotorState.ForwardAndTurnRight:
            this.stop();
            break;
          case MotorState.ForwardAndTurnLeft:
            this.stop();
            break;
          case MotorState.Back: {  // 前一刻后退,下一刻状态将变为后退向右
            if (this.motorLeft.speed == -MAX_SPEED)  // 若已满速
              this.motorRight.change(false, 3);  // 电机不能再加速，只需要右电机减速
            else
              this.acc(5, 3, false);
          }
            break;
          case MotorState.BackAndTurnRight: {
            if (this.motorLeft.speed == -MAX_SPEED) { // 若已满速
              // 左电机不能再加速，只需要右电机减速(需要考虑为0的状况)
              if (this.motorRight.speed < 0)
                this.motorRight.change(false, 5);
              else if (this.motorRight.speed == 0)
                this.motorRight.change(true, 5, true);  // 处理speed为0的情况，令状态进入BackRightCircle
            } else
              this.acc(5, 2, false);
          }
            break;
          case MotorState.BackAndTurnLeft: {
            if (this.motorRight.speed == -MAX_SPEED)  // 若已满速
            // 右电机不能再加速，只需要左电机加速(第三个参数处理speed为0的状况,且回正速度应更快)
              this.motorLeft.change(true, 7, false);
            else
              this.acc(6, 3, false);
          }
            break;
          case MotorState.ForwardRightCircle:
            this.stop();
            break;
          case MotorState.ForwardLeftCircle:
            this.stop();
            break;
          case MotorState.BackRightCircle:
            this.motorRight.change(true, 5);  // 已进入后退向右打转状态，此时左电机一定是反向满速状态，右电机正向，只需加速右电机
            break;
          case MotorState.BackdLeftCircle:
            this.stop();
            break;
          case MotorState.CWCircle:
            this.stop();
            break;
          case MotorState.CCWCircle:  // BackRightCircle状态持续到两个电机都满速则进入该状态
            break;
          default:
            break;
        }

      }
        break;
      case 0b0110: {  // 0110 下左
        switch (this.preMotorState) {
          case MotorState.Stop:
            this.acc(3, 5, false);  // 前一刻静止，则此刻立刻反向加速，但右电机加的较快，状态将变为后退向左
            break;
          case MotorState.Forward:
            this.stop();
            break;
          case MotorState.ForwardAndTurnRight:
            this.stop();
            break;
          case MotorState.ForwardAndTurnLeft:
            this.stop();
            break;
          case MotorState.Back: {  // 前一刻后退,下一刻状态将变为后退向左
            if (this.motorLeft.speed == -MAX_SPEED)  // 若已满速
              this.motorLeft.change(false, 3); // 电机不能再加速，只需要左电机减速
            else
              this.acc(3, 5, false);
          }
            break;
          case MotorState.BackAndTurnRight: {
            if (this.motorLeft.speed == -MAX_SPEED)  // 若已满速
            // 左电机不能再加速，只需要右电机加速(第三个参数处理speed为0的状况,且回正速度应更快)
              this.motorRight.change(true, 7, false);
            else
              this.acc(3, 6, false);
          }
            break;
          case MotorState.BackAndTurnLeft: {
            if (this.motorRight.speed == -MAX_SPEED) {  //若已满速
              // 右电机不能再加速，只需要左电机减速(需要考虑为0的状况)
              if (this.motorLeft.speed < 0)
                this.motorLeft.change(false, 5);
              else if (this.motorLeft.speed == 0)
                this.motorLeft.change(true, 5, true);  // 处理speed为0的情况，令状态进入BackLeftCircle
            }
            else
              this.acc(2, 5, false);

          }
            break;
          case MotorState.ForwardRightCircle:
            this.stop();
            break;
          case MotorState.ForwardLeftCircle:
            this.stop();
            break;
          case MotorState.BackRightCircle:
            this.stop();
            break;
          case MotorState.BackdLeftCircle:
            this.motorLeft.change(true, 5);  // 已进入后退向左打转状态，此时右电机一定是反向满速状态，左电机正向，只需加速左电机
            break;
          case MotorState.CWCircle:  // BackLeftCircle状态持续到两个电机都满速则进入该状态
            break;
          case MotorState.CCWCircle:
            this.stop();
            break;
          default:
            break;
        }

      }
        break;
      case 0b0111: {  //0111 下左右 按键冲突，视为下
        this._changeSpeedByKey(4)
      }
        break
      case 0b1000: {  //1000 上
        switch (this.preMotorState) {
          case MotorState.Stop:
            this.dualAcc(3, 3, true);  // 前一刻停止，则此刻直接双轮正向加速
            break;
          case MotorState.Forward:
            this.dualAcc(3, 3, true);  // 前一刻前进，则此刻直接双轮正向加速
            break;
          case MotorState.ForwardAndTurnRight:
            this.dualAcc(3, 5, true);  // 前一刻前进向右，则此刻双轮正向加速，但右电机加速应更快，需回正
            break;
          case MotorState.ForwardAndTurnLeft:
            this.dualAcc(5, 3, true);  // 前一刻前进向左，则此刻双轮正向加速，但左电机加速应更快，需回正
            break;
          case MotorState.Back:
            this.stop();  // 前一刻后退，则此刻立刻停止后直接双轮正向加速
            break;
          case MotorState.BackAndTurnRight:
            this.stop();  // 前一刻后退向右转，则此刻立刻停止后直接双轮正向加速
            break;
          case MotorState.BackAndTurnLeft:
            this.stop();  // 前一刻后退向左转，则此刻立刻停止后直接双轮正向加速
            break;
          case MotorState.ForwardRightCircle:
            this.motorRight.speed = 0;  // 前一刻前进右打转，左电机必定正向满速，右电机反向，则此刻立刻归零右电机
            break;
          case MotorState.ForwardLeftCircle:
            this.motorLeft.speed = 0;  // 前一刻前进左打转，右电机必定正向满速，左电机反向，则此刻立刻归零左电机
            break;
          case MotorState.BackRightCircle:
            this.stop();  // 前一刻后退右打转，则此刻立刻停止后直接双轮正向加速
            break;
          case MotorState.BackdLeftCircle:
            this.stop();  // 前一刻后退左打转，则此刻立刻停止后直接双轮正向加速
            break;
          case MotorState.CWCircle:
            this.stop();  // 前一刻顺时针原地打转，则此刻立刻停止后直接双轮正向加速
            break;
          case MotorState.CCWCircle:
            this.stop();  // 前一刻逆时针原地打转，则此刻立刻停止后直接双轮正向加速
            break;
          default:
            break;
        }
      }
        break;
      case 0b1001: {  //1001 上右
        switch (this.preMotorState) {
          case MotorState.Stop:
            this.acc(5, 3, true);  // 前一刻静止，则此刻立刻正向加速，但左电机加的较快，状态将变为前进向右
            break;
          case MotorState.Forward: {  // 前一刻前进,下一刻状态将变为前进向右
            if (this.motorLeft.speed == MAX_SPEED)  // 若已满速
              this.motorRight.change(false, 3);  // 电机不能再加速，只需要右电机减速
            else
              this.acc(5, 3, true);
          }
            break;
          case MotorState.ForwardAndTurnRight: {
            if (this.motorLeft.speed == MAX_SPEED) {  // 若已满速
              // 左电机不能再加速，只需要右电机减速(需要考虑为0的状况)
              if (this.motorRight.speed > 0)
                this.motorRight.change(false, 5);
              else if (this.motorRight.speed == 0)
                this.motorRight.change(true, 5, false);  // 处理speed为0的情况，令状态进入ForwardRightCircle
            } else
              this.acc(5, 2, true);
          }
            break;
          case MotorState.ForwardAndTurnLeft: {
            if (this.motorRight.speed == MAX_SPEED)  // 若已满速
            // 右电机不能再加速，只需要左电机加速(第三个参数处理speed为0的状况,且回正速度应更快)
              this.motorLeft.change(true, 7, true);
            else
              this.acc(6, 3, true);
          }
            break;
          case MotorState.Back:
            this.stop();
            break;
          case MotorState.BackAndTurnRight:
            this.stop();
            break;
          case MotorState.BackAndTurnLeft:
            this.stop();
            break;
          case MotorState.ForwardRightCircle:
            this.motorRight.change(true, 5);  // 已进入前进向右打转状态，此时左电机一定是正向满速状态，右电机反向，只需加速右电机
            break;
          case MotorState.ForwardLeftCircle:
            this.stop();
            break;
          case MotorState.BackRightCircle:
            this.stop();
            break;
          case MotorState.BackdLeftCircle:
            this.stop();
            break;
          case MotorState.CWCircle:  // ForwardRightCircle状态持续到两个电机都满速则进入该状态
            break;
          case MotorState.CCWCircle:
            this.stop();
            break;
          default:
            break;
        }

      }
        break;
      case 0b1010: {  //1010 上左
        switch (this.preMotorState) {
          case MotorState.Stop:
            this.acc(3, 5, true);  // 前一刻静止，则此刻立刻正向加速，但右电机加的较快，状态将变为前进向左
            break;
          case MotorState.Forward: {  // 前一刻前进,下一刻状态将变为前进向左
            if (this.motorRight.speed == MAX_SPEED)  // 若已满速
              this.motorLeft.change(false, 3);  // 电机不能再加速，只需要左电机减速
            else
              this.acc(3, 5, true);
          }
            break;
          case MotorState.ForwardAndTurnRight: {
            if (this.motorLeft.speed == MAX_SPEED)  // 若已满速
            // 左电机不能再加速，只需要右电机加速(第三个参数处理speed为0的状况,且回正速度应更快)
              this.motorRight.change(true, 7, true);
            else
              this.acc(3, 6, true);

          }
            break;
          case MotorState.ForwardAndTurnLeft: {
            if (this.motorRight.speed == MAX_SPEED) {  // 若已满速
              // 右电机不能再加速，只需要左电机减速(需要考虑为0的状况)
              if (this.motorLeft.speed > 0)
                this.motorLeft.change(false, 5);
              else if (this.motorLeft.speed == 0)
                this.motorLeft.change(true, 5, false);  // 处理speed为0的情况，令状态进入ForwardLeftCircle
            } else
              this.acc(2, 5, true);
          }
            break;
          case MotorState.Back:
            this.stop();
            break;
          case MotorState.BackAndTurnRight:
            this.stop();
            break;
          case MotorState.BackAndTurnLeft:
            this.stop();
            break;
          case MotorState.ForwardRightCircle:
            this.stop();
            break;
          case MotorState.ForwardLeftCircle:
            this.motorLeft.change(true, 5);  // 已进入前进向左打转状态，此时右电机一定是正向满速状态，左电机反向，只需加速左电机
            break;
          case MotorState.BackRightCircle:
            this.stop();
            break;
          case MotorState.BackdLeftCircle:
            this.stop();
            break;
          case MotorState.CWCircle:
            this.stop();
            break;
          case MotorState.CCWCircle:  // ForwardRightCircle状态持续到两个电机都满速则进入该状态
            break;
          default:
            break;
        }

      }
        break;
      case 0b1011: {  //1011 上 左右 按键冲突，视为上
        this._changeSpeedByKey(0b1000)
      }
        break
      case 0b1100: {  //1100 上下 按键冲突，视为停止
        this._changeSpeedByKey(0b0000)
      }
        break
      case 0b1101: {  //1101 上下右 按键冲突，视为右
        this._changeSpeedByKey(0b0001)
      }
        break
      case 0b1110: {  //1110 上下左 按键冲突，视为左
        this._changeSpeedByKey(0b0010)
      }
        break
      case 0b1111: {  //1111 上下左右 按键冲突，视为停止
        this._changeSpeedByKey(0b0000)
      }
        break
      default:
        break;
    }
  }

  /**
   * 更新当前小车状态
   */
  private updateMotorState () {
    if (this.motorLeft.speed > 0) {  // 左电机正转
      if (this.motorRight.speed > 0) {  // 右电机正转
        if (this.motorLeft.speed > this.motorRight.speed)
          this.curMotorState = MotorState.ForwardAndTurnRight
        else if (this.motorLeft.speed < this.motorRight.speed)
          this.curMotorState = MotorState.ForwardAndTurnLeft
        else
          this.curMotorState = MotorState.Forward
      } else if (this.motorRight.speed < 0) {  // 右电机反转
        if (this.motorLeft.speed > Math.abs(this.motorRight.speed))
          this.curMotorState = MotorState.ForwardRightCircle
        else if (this.motorLeft.speed < Math.abs(this.motorRight.speed))
          this.curMotorState = MotorState.BackdLeftCircle
        else
          this.curMotorState = MotorState.CWCircle
      } else {  //右电机静止
        this.curMotorState = MotorState.ForwardAndTurnRight
      }
    } else if (this.motorLeft.speed < 0) {  //左电机反转
      if (this.motorRight.speed > 0) {  //右电机正转
        if (Math.abs(this.motorLeft.speed) > this.motorRight.speed)
          this.curMotorState = MotorState.BackRightCircle
        else if (Math.abs(this.motorLeft.speed) < this.motorRight.speed)
          this.curMotorState = MotorState.ForwardLeftCircle
        else
          this.curMotorState = MotorState.CCWCircle
      } else if (this.motorRight.speed < 0) {  //右电机反转
        if (this.motorLeft.speed > this.motorRight.speed)
          this.curMotorState = MotorState.BackAndTurnLeft
        else if (this.motorLeft.speed < this.motorRight.speed)
          this.curMotorState = MotorState.BackAndTurnRight
        else
          this.curMotorState = MotorState.Back
      } else {  // 右电机静止
        this.curMotorState = MotorState.BackAndTurnRight
      }
    } else {  // 左电机静止
      if (this.motorRight.speed > 0) {  // 右电机正转
        this.curMotorState = MotorState.ForwardAndTurnLeft
      } else if (this.motorRight.speed < 0) {  // 右电机反转
        this.curMotorState = MotorState.BackAndTurnLeft
      } else {  // 右电机静止
        this.curMotorState = MotorState.Stop
      }
    }
  }

  /**
   * 电机立刻停止
   */
  private stop () {
    this.motorLeft.speed = 0
    this.motorRight.speed = 0

  }

  /**
   * 不回正的减速
   * @param lstep
   * @param rstep
   */
  private dec (lstep: number, rstep: number) {
    this.motorLeft.change(false, lstep)
    this.motorRight.change(false, rstep)
  }

  /**
   * 不回正的加速
   * @param lstep 左电机加速的步长
   * @param rstep 右电机加速的步长
   * @param forward 是否正向加速
   */
  private acc (lstep: number, rstep: number, forward: boolean) {
    this.motorLeft.change(true, lstep, forward)
    this.motorRight.change(true, rstep, forward)
  }

  /**
   * 双轮转向一致时的减速，将会首先减速回正，再同时减速
   * @param lstep 左电机正向减速的步长
   * @param rstep 右电机正向减速的步长
   */
  private dualDec (lstep: number, rstep: number) {
    if (lstep > rstep) {  // 此时说明我们需要左轮快点减速与右轮速度一致，进行回正操作
      if (Math.abs(this.motorLeft.speed) - lstep >= Math.abs(this.motorLeft.speed) - rstep) {  // 左轮仍比右轮快
        this.motorLeft.change(false, lstep)
        this.motorRight.change(false, rstep)
      } else {  // 左轮已经与右轮速度减至基本相同,将他们速度设置一致，过后状态将会为forward
        this.motorLeft.speed = this.motorRight.speed
        this.motorLeft.change(false, rstep)
        this.motorRight.change(false, rstep)
      }
    } else {
      if (Math.abs(this.motorLeft.speed) - lstep <= Math.abs(this.motorLeft.speed) - rstep) {  // 左轮仍比右轮慢
        this.motorLeft.change(false, lstep)
        this.motorRight.change(false, rstep)
      }
      else {  // 左轮已经与右轮速度减至基本相同,将他们速度设置一致，过后状态将会为forward
        this.motorRight.speed = this.motorLeft.speed
        this.motorLeft.change(false, lstep)
        this.motorRight.change(false, lstep)
      }
    }

  }

  /**
   * 双轮转向一致时的加速，将会加速回正，并同时加速
   * @param lstep 左电机加速的步长
   * @param rstep 右电机加速的步长
   * @param forward 是否正向加速
   */
  private dualAcc (lstep: number, rstep: number, forward: boolean) {
    if (forward) {  // 正向
      if (lstep > rstep) {  // 此时说明我们需要左轮快点正向加速与右轮速度一致，进行回正操作
        if (this.motorLeft.speed + lstep <= this.motorRight.speed + rstep) {  // 左轮仍比右轮慢
          this.motorLeft.change(true, lstep, true)
          this.motorRight.change(true, rstep, true)
        } else {  // 左轮已经与右轮速度加至基本相同,将他们速度设置一致，过后状态将会为forward
          this.motorLeft.speed = this.motorRight.speed
          this.motorLeft.change(true, rstep, true)
          this.motorRight.change(true, rstep, true)
        }
      } else {  // 此时说明我们需要右轮快点正向加速与左轮速度一致，此时进行回正操作
        if (this.motorLeft.speed + lstep >= this.motorRight.speed + rstep) {
          this.motorLeft.change(true, lstep, true)
          this.motorRight.change(true, rstep, true)
        } else {  // 左轮已经与右轮速度加至基本相同,将他们速度设置一致，过后状态将会为forward
          this.motorRight.speed = this.motorLeft.speed
          this.motorLeft.change(true, lstep, true)
          this.motorRight.change(true, lstep, true)
        }
      }
    } else {  // 反向
      if (lstep > rstep) {  // 此时说明我们需要左轮快点反向加速与右轮速度一致，进行回正操作
        if (this.motorLeft.speed - lstep >= this.motorRight.speed - rstep) {  // 左轮仍比右轮慢
          this.motorLeft.change(true, lstep, false)
          this.motorRight.change(true, rstep, false)
        } else {  // 左轮已经与右轮速度加至基本相同,将他们速度设置一致，过后状态将会为back
          this.motorLeft.speed = this.motorRight.speed
          this.motorLeft.change(true, rstep, false)
          this.motorRight.change(true, rstep, false)
        }
      } else {  // 此时说明我们需要右轮快点反向加速与左轮速度一致，此时进行回正操作
        if (this.motorLeft.speed - lstep <= this.motorRight.speed - rstep) {
          this.motorLeft.change(true, lstep, false)
          this.motorRight.change(true, rstep, false)
        } else {  // 左轮已经与右轮速度加至基本相同,将他们速度设置一致，过后状态将会为forward
          this.motorRight.speed = this.motorLeft.speed
          this.motorLeft.change(true, lstep, false)
          this.motorRight.change(true, lstep, false)
        }
      }
    }
  }

}

/**
 * 一个电机
 */
class Motor {
  public speed: number = 0

  /**
   * 有渐变地改变电机速度
   * @param accelerating 是否加速
   * @param step 变化步长
   * @param forward 该参数处理前一刻电机静止时的状况，speed为0时且accelerating为true时有效。即静止时正转加速为true，静止时反转加速为false
   */
  change (accelerating: boolean, step: number, forward: boolean = true) {
    if (this.speed > 0) {  // 前一刻正转
      if (accelerating) {  // 若需要加速
        if (this.speed + step > MAX_SPEED)  // 越界处理
          this.speed = MAX_SPEED
        else
          this.speed += step
      }
      else {  // 若需要减速
        if (this.speed - step < 0)  // 越界处理
          this.speed = 0
        else
          this.speed -= step
      }
    } else if (this.speed < 0) {  // 前一刻反转
      if (accelerating) {  // 若需要加速
        if (this.speed - step < -MAX_SPEED)  // 越界处理
          this.speed = -MAX_SPEED
        else
          this.speed -= step
      } else {  // 若需要减速
        if (this.speed + step > 0)  // 越界处理
          this.speed = 0
        else
          this.speed += step
      }
    } else {  // 前一刻this.speed为0，需要根据参数accelerating和forward确定，accelerating为false时继续保持静止
      if (accelerating) {  // 若需要加速
        if (forward) this.speed = step  // 前进
        else this.speed = -step  //后退
      }
    }
  }
}


export enum MotorState { // 状态枚举
  /**
   * 静止，此时两电机应趋向0
   * @type {number}
   */
  Stop = 0,
    /**
     * 前进，此时两电机速度大于0，两电机速度相同
     * @type {number}
     */
  Forward = 1,
    /**
     * 前进右转，此时电机0和电机1的速度大于0，电机0的速度要大于电机1
     * @type {number}
     */
  ForwardAndTurnRight = 2,
    /**
     * 前进左转，此时电机0和电机1的速度大于0，电机1的速度要大于电机0
     * @type {number}
     */
  ForwardAndTurnLeft = 3,
    /**
     * 后退，此时两电机速度小于0，两电机速度相同
     * @type {number}
     */
  Back = 4,
    /**
     * 后退右转，此时电机0和电机1的速度小于0，电机0的速度的绝对值要大于电机1
     * @type {number}
     */
  BackAndTurnRight = 5,
    /**
     * 后退左转，此时电机0和电机1的速度小于0，电机1的速度的绝对值要大于电机0
     * @type {number}
     */
  BackAndTurnLeft = 6,
    /**
     * 前右打转，此时电机0的速度应大于0，电机1的速度应小于等于0，但电机0的速度的绝对值要远大于1的
     * @type {number}
     */
  ForwardRightCircle = 7,
    /**
     * 前左打转，此时电机1的速度应大于0，电机0的速度应小于等于0，但电机1的速度的绝对值要远大于0的
     * @type {number}
     */
  ForwardLeftCircle = 8,
    /**
     * 后右打转，此时电机0的速度应小于0，电机1的速度应大于等于0，但电机0的速度的绝对值要远大于1的
     * @type {number}
     */
  BackRightCircle = 9,
    /**
     * 后左打转，此时电机1的速度应小于0，电机0的速度应大于等于0，但电机1的速度的绝对值要远大于0的
     * @type {number}
     */
  BackdLeftCircle = 10,
    /**
     * 原地顺时针打转
     * @type {number}
     */
  CWCircle = 11,
    /**
     * 原地逆时针打转
     * @type {number}
     */
  CCWCircle = 12
}
