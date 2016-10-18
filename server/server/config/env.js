"use strict";
const path = require("path")
const _ = require("lodash")

const root_dir = process.env.ROOT_DIR || path.join(__dirname, "/..")
const public_dir = process.env.PUBLIC_DIR || path.resolve(__dirname, '..', 'public')
const log_dir = process.env.LOG_DIR || path.resolve(__dirname, '..', '..', 'logs')
const node_env = process.env.NODE_ENV || "development"

let base = {
  app: {
    root_dir,
    public_dir,
    log_dir,
    node_env
  }
}

let specific = {
  development: {
    app: {
      port: 5000,
      name: "koa2",
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
      port: process.env.PORT || 5000,
      name: "koa2",
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

module.exports = _.merge(base, specific[node_env])