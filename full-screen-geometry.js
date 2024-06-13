import { BufferGeometry, Float32BufferAttribute } from 'three'

export class FullScreenGeometry extends BufferGeometry {
  constructor() {
    super()
    this.deleteAttribute('normal')
    this.deleteAttribute('uv')
    this.setAttribute('position', new Float32BufferAttribute([1, 1, 0, -1, -1, 0, 1, -1, 0, 1, 1, 0, -1, 1, 0, -1, -1, 0], 3))
    this.setAttribute('uv', new Float32BufferAttribute([1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0], 2))
  }
}
