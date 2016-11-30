import './app.loader.ts'
import {
  Component,
  ViewEncapsulation,
  ViewContainerRef,
  OnInit,
  AfterViewInit
} from '@angular/core'
import { GlobalState } from './global.state'
import { BaImageLoaderService, BaThemePreloader, BaThemeSpinner } from './theme/services'
import { layoutPaths } from './theme/theme.constants'
import { BaThemeConfig } from './theme/theme.config'
import { WebSocketService } from './service-share/services'

/*
 * App Component
 * Top Level Component
 */
@Component({
  selector: 'app',
  encapsulation: ViewEncapsulation.None,
  styles: [require('normalize.css'), require('./app.scss')],
  template: `
    <main [ngClass]="{'menu-collapsed': isMenuCollapsed}" baThemeRun>
      <div class="additional-bg"></div>
      <router-outlet></router-outlet>
    </main>
  `
})
export class App implements OnInit, AfterViewInit {

  isMenuCollapsed: boolean = false

  constructor (private _state: GlobalState,
               private _imageLoader: BaImageLoaderService,
               private _spinner: BaThemeSpinner,
               private _config: BaThemeConfig,
               private viewContainerRef: ViewContainerRef,
               private _wsService: WebSocketService) {

    this._loadImages()

    this._state.subscribe('menu.isCollapsed', (isCollapsed) => {
      this.isMenuCollapsed = isCollapsed
    })
  }

  ngOnInit () {
    this._wsService.connect()
  }

  public ngAfterViewInit (): void {
    // hide spinner once all loaders are completed
    BaThemePreloader.load().then((values) => {
      this._spinner.hide()
    })
  }

  private _loadImages (): void {
    // register some loaders
    BaThemePreloader.registerLoader(this._imageLoader.load(layoutPaths.images.root + 'sky-bg.jpg'))
  }
}
