'use strict'

import Router from 'koa-router'

import routerMain from './main'
import routerAuth from './auth'
import routerMock from './mock'

const router = new Router()


router.use('/', routerMain.routes())
router.use('/auth', routerAuth.routes())
router.use('/mock', routerMock.routes())

router.get('*', async (ctx, next) => {
  ctx.body = {status: 404}
})

export default function applyRoutes (app) {
  app
    .use(router.routes())
    .use(router.allowedMethods())
}
