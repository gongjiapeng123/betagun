
import webSocketServer from 'socket.io'

export default function configWebsocket (server) {
  const io = webSocketServer(server)
  console.log('配置WebSocket')
  io.on('connection', (socket) => {  // 客户端WebSocket连接上服务器时
    console.log(`客户端 ${socket.handshake.headers.origin} 已连接WebSocket`)

    // 该客户端对数据的订阅，客户端都会订阅数据
    const jy901Subscription = observalbes.jy901Observable.subscribe(
      (jy901Data) => {
        socket.emit('jy901', {jy901Data: jy901Data})
      }
    )
    const arduinoSubscription = observalbes.arduinoObservable.subscribe(
      (arduinoData) => {
        socket.emit('arduino', {arduinoData: arduinoData})
      }
    )

    socket.emit('welcome', {welcome: 'welcome, my friend!'})  // 发送欢迎事件给客户端

    socket
      .on('disconnect', () => {  // 连接断开

      })

  })
}
