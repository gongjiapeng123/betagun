"use strict";
const path = require("path")
const _ = require("lodash")

const NODE_ENV = process.env.NODE_ENV || "development"
const ROOT_DIR = path.join(__dirname, "/..")
const PUBLIC_DIR = path.resolve(__dirname, '..', '..', 'frontend', 'dist')
const LOG_DIR = path.resolve(__dirname, '..', '..', 'logs')


let base = {
  app: {
    name: 'betagun node server',
  },
  ROOT_DIR,
  PUBLIC_DIR,
  LOG_DIR,
  NODE_ENV
}

let specific = {
  development: {
    app: {
      port: 61616,
      excluded: "excluded_path"
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
    app: {
      port: process.env.PORT || 61616,
      excluded: "excluded_path"
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

module.exports = _.merge(base, specific[NODE_ENV])