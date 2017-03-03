import {
  Component,
  ViewEncapsulation,
  OnInit,
  OnDestroy,
} from '@angular/core'

import { Observable } from 'rxjs/Observable'
import { Subscription } from 'rxjs/Subscription'
import { GlobalState } from '../../../global.state'
import { WebSocketService } from '../../../service-share/services'

import 'style-loader!./baPageTop.scss'

@Component({
  selector: 'ba-page-top',
  templateUrl: './baPageTop.html',
  encapsulation: ViewEncapsulation.None,
})
export class BaPageTop implements OnInit, OnDestroy {

  isAuth: boolean = false
  isAuthSubscription: Subscription

  public isScrolled: boolean = false
  public isMenuCollapsed: boolean = false

  constructor (private _state: GlobalState,
               private _wsService: WebSocketService) {
  }

  ngOnInit () {
    // 开发时自动登录
    // const env = process.env.NODE_ENV
    // if (env === 'development') {
    //   this._wsService.login('admin', '666')
    // }

    // 自动登录
    this._wsService.login('admin', '666')

    this._state.subscribe('menu.isCollapsed', (isCollapsed) => {
      this.isMenuCollapsed = isCollapsed
    })

    this.isAuthSubscription = this._wsService.isAuth$.subscribe(isAuth => {
      this.isAuth = isAuth
    })
  }

  ngOnDestroy () {
    this.isAuthSubscription && this.isAuthSubscription.unsubscribe()
  }

  public toggleMenu () {
    this.isMenuCollapsed = !this.isMenuCollapsed
    this._state.notifyDataChanged('menu.isCollapsed', this.isMenuCollapsed)
    return false
  }

  public scrolledChanged (isScrolled) {
    this.isScrolled = isScrolled
  }

  public logout () {
    this._wsService.logout()
  }
}
