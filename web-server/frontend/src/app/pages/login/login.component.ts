import {
  Component,
  ViewEncapsulation,
  OnInit,
  OnDestroy
} from '@angular/core'
import { Router } from '@angular/router'
import { FormGroup, AbstractControl, FormBuilder, Validators } from '@angular/forms'

import { WebSocketService } from '../../service-share/services'
import { Subscription } from 'rxjs/Subscription'

@Component({
  selector: 'login',
  encapsulation: ViewEncapsulation.None,
  styles: [require('./login.scss')],
  template: require('./login.html'),
})
export class Login implements OnInit, OnDestroy {

  public form: FormGroup
  public username: AbstractControl
  public password: AbstractControl
  public submitted: boolean = false

  loginResultSubscription: Subscription
  authError: boolean = false

  constructor (fb: FormBuilder,
               private router: Router,
               private _wsService: WebSocketService) {
    this.form = fb.group({
      'username': ['admin', Validators.compose([Validators.required, Validators.minLength(4)])],
      'password': ['', Validators.compose([Validators.required, Validators.minLength(3)])]
    })

    this.username = this.form.controls['username']
    this.password = this.form.controls['password']
  }

  ngOnInit () {
    this.loginResultSubscription = this._wsService.loginResult$
      .subscribe((rs: {succeeded: boolean, username: string, action: string}) => {
        this.authError = !rs.succeeded
        if (rs.succeeded) {
          this.router.navigate(['/pages'])
        }
      })
  }

  ngOnDestroy () {
    this.loginResultSubscription && this.loginResultSubscription.unsubscribe()
  }

  public onSubmit (values: any): void {
    this.submitted = true
    if (this.form.valid) {
      this._wsService.login(values.username, values.password)
    }
  }
}
