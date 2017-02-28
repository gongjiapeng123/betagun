'use strict';
const path = require('path')
const _ = require('lodash')

const NODE_ENV = process.env.NODE_ENV || 'development'
const IS_DEV = NODE_ENV === 'development'
const ROOT_DIR = path.resolve(__dirname, '../..')
const SERVER_DIR = path.resolve(ROOT_DIR, 'server')
const SERVER_PUBLIC_DIR = path.resolve(SERVER_DIR, 'public')
const FRONTEND_DIR = path.resolve(ROOT_DIR, 'frontend')
const FRONTEND_DIST_DIR = path.resolve(FRONTEND_DIR, 'dist')
const LOG_DIR = path.resolve(ROOT_DIR, 'logs')


const base = {
  app: {
    name: 'betagun node server',
  },
  NODE_ENV,
  IS_DEV,
  ROOT_DIR,
  SERVER_DIR,
  SERVER_PUBLIC_DIR,
  FRONTEND_DIR,
  FRONTEND_DIST_DIR,
  LOG_DIR,
}

const specific = {
  development: {
    http_port: 61620,
    // tcp_host: '192.168.66.107',  // tcp 服务器监听地址
    tcp_host: '0.0.0.0',  // tcp 服务器监听地址
    app: {
      excluded: 'excluded_path',
    },
    db: {
      host: 'localhost',
      port: 27017,
      user: 'admin',
      password: 'admin',
      database: 'admin'
    }
  },
  production: {
    port: process.env.PORT || 61620,
    tcp_host: '127.0.0.1',
    app: {
      excluded: 'excluded_path',
    },
    db: {
      host: 'localhost',
      port: 27017,
      user: 'admin',
      password: 'admin',
      database: 'admin'
    }
  },
};

export const env = _.merge(base, specific[NODE_ENV])