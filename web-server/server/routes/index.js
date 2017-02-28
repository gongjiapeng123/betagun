'use strict'

import Router from 'koa-router'

import routerMain from './main'
import routerAuth from './auth'
import routerMock from './mock'

const router = new Router()


router.use('/auth', routerAuth.routes())
router.use('/mock', routerMock.routes())
router.use('/', routerMain.routes())

// // 所有都返回 /
// router.get('*', async (ctx, next) => {
//   ctx.redirect('/')
// })

router.use(async (ctx, next) => {
  ctx.sendFile("index.html");
})

export function applyRoutes (app) {
  app
    .use(router.routes())
    .use(router.allowedMethods())
}
