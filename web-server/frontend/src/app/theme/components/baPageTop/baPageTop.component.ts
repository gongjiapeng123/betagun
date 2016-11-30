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

@Component({
  selector: 'ba-page-top',
  styles: [require('./baPageTop.scss')],
  template: require('./baPageTop.html'),
  encapsulation: ViewEncapsulation.None
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
