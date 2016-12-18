import {
  Component,
  ViewEncapsulation,
  OnInit,
  OnDestroy,
} from '@angular/core'
import { WebSocketService } from  '../../service-share/services'
import { Subscription } from 'rxjs/Subscription'


@Component({
  selector: 'dashboard',
  encapsulation: ViewEncapsulation.None,
  styles: [require('./control-board.component.scss')],
  template: require('./control-board.component.html')
})
export class ControlBoard implements OnInit, OnDestroy  {
  loginUser: string
  private _loginUserSubscription: Subscription

  constructor (private _wsService: WebSocketService) {
  }

  ngOnInit () {
    this._loginUserSubscription = this._wsService.loginUser$
      .subscribe(loginUser => {
        this.loginUser = loginUser
      })
  }

  ngOnDestroy () {
    this._loginUserSubscription.unsubscribe()
  }

}
