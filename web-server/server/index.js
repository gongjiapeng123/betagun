'use strict'

import http from 'http'
import Koa from 'koa'
import applyMiddleware from './middleware'
import applyRoutes from './routes'
import { configLog, appInit, env, configWebSocket } from './config'

const app = new Koa()

// 日志
const logger = configLog()
app.context.logger = logger  // logger挂载在ctx

// 常规中间件
appInit(app)

// 自定义中间件
applyMiddleware(app)

// 路由
applyRoutes(app)


import './config/client4tcp'  // 先启动tcp客户端
const server = http.createServer(app.callback())
configWebSocket(server)
server.listen(env.app.port)

logger.info("Server started, listening on port: " + env.app.port)

export default app