const webpack = require('webpack')
const path = require('path')
const fs = require('fs')

let nodeModules = {}
fs.readdirSync('node_modules')
  .filter(function(x) {
    return ['.bin'].indexOf(x) === -1
  })
  .forEach(function(mod) {
    nodeModules[mod] = 'commonjs ' + mod
  })


module.exports = {
  devtool: 'sourcemap',
  target: 'node',
  externals: nodeModules,

  resolve: {
    extensions: ['', '.js', '.json']
  },

  entry: ['babel-polyfill', './server'],

  output: {
    path: 'dist',
    filename: 'server.js'
  },

  module: {
    loaders: [
      {
        test: /\.json$/,
        loader: 'json'
      },
      {
        test: /\.js$/,
        loader: 'babel',
        exclude: [
          './server/public',
        ]
      },
    ]
  },

  plugins: [

    new webpack.DefinePlugin({
      'process.env': {
        NODE_ENV: JSON.stringify('production'),
        ROOT_DIR: JSON.stringify(path.resolve(__dirname, 'dist')),
        PUBLIC_DIR: JSON.stringify(path.resolve(__dirname, 'server', 'public')),
        LOG_DIR: JSON.stringify(path.resolve(__dirname, 'logs'))
      },
    })
  ],
}