

import compose from 'koa-compose'
import checkAuth from './checkAuth'

export function applyMiddleware (app) {
  app.use(compose([
    // checkAuth(),  // have to login
  ]))
}