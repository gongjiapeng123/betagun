import { sendControlCommand } from './client4tcp'
import webSocketServer from 'socket.io'
import * as observables from './observables'
import log4js from 'log4js'
const logger = log4js.getLogger('default')


/**
 * 验证用户，小应用就不用数据库了，这里不设置服务器的guess用户，让其他客户端一打开网页就能看到数据显示
 */
function checkUser(username, password) {
  let users = new Map().set('admin', '666')
  return users.has(username) && users.get(username) == password
}

export default function configWebsocket (server) {
  const io = webSocketServer(server)
  logger.log('配置WebSocket')
  io.on('connection', (socket) => {  // 客户端WebSocket连接上服务器时
    logger.log(`客户端 ${socket.handshake.headers.origin} 已连接WebSocket`)

    let user = 'nobody'  // 该客户端的用户名
    let imageSubscription = null  // 该客户端对图像的订阅

    // 该客户端对数据的订阅，客户端都会订阅数据
    const jy901Subscription = observables.jy901$.subscribe(
      (jy901Data) => {
        socket.emit('jy901', {jy901Data})  // data: {jy901Data: string}
      }
    )
    const arduinoSubscription = observables.arduino$.subscribe(
      (arduinoData) => {
        socket.emit('arduino', {arduinoData})  // data: {arduinoData: string}
      }
    )
    const infoSubscription = observables.info$.subscribe(
      (infoData) => {  // 小车计算结果数据
        socket.emit('info', infoData)  // data: {type: string, info: string}
      }
    )

    // 发送欢迎事件给客户端
    socket.emit('welcome', {welcome: 'welcome, my friend!'})

    socket
      .on('disconnect', () => {  // 连接断开
        jy901Subscription && jy901Subscription.unsubscribe()
        arduinoSubscription && arduinoSubscription.unsubscribe()
        imageSubscription && imageSubscription.unsubscribe()
        infoSubscription && infoSubscription.unsubscribe()
      })
      .on('login', (data) => {  // 获得客户端传来的login事件，data:{username:string, password:string}
        const succeeded = checkUser(data.username, data.password)

        if (succeeded) {
          user = data.username
        }
        // 发送登录结果事件给客户端
        socket.emit('login_rs', {succeeded: succeeded, username: data.username, action: 'login'})

      })
      .on('logout', (data) => {  // 获得客户端传来的login事件，data:{username:string}
        user = 'nobody'
        socket.emit('logout_rs', {succeeded: true, username: '', action: 'logout'})
      })
      .on('imageOn', () => {  // 客户端要求获取图像数据
        if (user == 'admin') {
          imageSubscription = observables.image$.subscribe(  // 订阅
            (image) => {  // 传来的是图片的二进制数据
              // console.log('emit a image!')

              // 发送给浏览器客户端，客户端需要Uint8Array来保存图片
              socket.emit('image', Buffer.from(image, 'binary'))
            }
          )
        }
      })
      .on('imageOff', () => {  // 客户端关闭获取图像
        imageSubscription && imageSubscription.unsubscribe()  // 取消订阅
      })
      .on('motorsControl', (data) => {  // 获得客户端发送过来的motor命令，data:{motorLeftSpeed:number, motorRightSpeed:number}
        // console.log(`Client send motor cmd: ${data.motorLeftSpeed} ${data.motorRightSpeed}`);
        sendControlCommand('motorsControl', data)
      })

  })
}
