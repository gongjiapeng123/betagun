
import webSocketServer from 'socket.io'

export default function configWebsocket (server) {
  const io = webSocketServer(server)

  io.on('connection', (socket) => {  // 客户端WebSocket连接上服务器时
    console.log('client connected')
    socket.emit('welcome', {welcome: 'welcome, my friend!'})  // 发送欢迎事件给客户端

    socket
      .on('disconnect', () => {  // 连接断开

      })

  })
}
