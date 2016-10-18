

import compose from 'koa-compose'
import checkAuth from './checkAuth'

export default function applyMiddleware (app) {
  app.use(compose([
    // checkAuth(),  // have to login
  ]))
}