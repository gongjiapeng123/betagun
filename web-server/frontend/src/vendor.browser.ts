// For vendors for example jQuery, Lodash, angular2-jwt just import them here unless you plan on
// chunking vendors files for async loading. You would need to import the async loaded vendors
// at the entry point of the async loaded file. Also see custom-typings.d.ts as you also need to
// run `typings install x` where `x` is your module

// Angular 2
import '@angular/platform-browser'
import '@angular/platform-browser-dynamic'
import '@angular/core'
import '@angular/common'
import '@angular/forms'
import '@angular/http'
import '@angular/router'

// AngularClass
import '@angularclass/hmr'

// RxJS
import 'rxjs/add/operator/map'
import 'rxjs/add/operator/mergeMap'
import 'rxjs/add/operator/switchMap'
import 'rxjs/add/operator/catch'
import 'rxjs/add/operator/distinctUntilChanged'
import 'rxjs/observable/of'
import 'rxjs/observable/fromEvent'
import 'rxjs/add/operator/sampleTime'

// Web dependencies
import 'jquery'
import 'bootstrap-loader'
import 'font-awesome-sass-loader'
import 'lodash'
import 'fabric'

if ('production' === ENV) {
  // Production
} else {
  // Development
}
