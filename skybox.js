import { BoxGeometry, BackSide, Mesh, ShaderMaterial, CubeTextureLoader } from 'three'

export class SkyBox extends Mesh {
  constructor() {
    const geometry = new BoxGeometry(100, 100, 100)
    geometry.deleteAttribute('normal')
    geometry.deleteAttribute('uv')
    const material = new ShaderMaterial({
      side: BackSide,
      fog: false,
      transparent: true,
      depthTest: false,
      depthWrite: false,
      uniforms: {
        tCube: { value: null }
      },
      vertexShader: `
        varying vec3 rayDir;
        void main() {
          rayDir = normalize(position - cameraPosition);
          gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.);
        }
      `,
      fragmentShader: `
        varying vec3 rayDir;
        uniform samplerCube tCube;
        void main() {
          vec4 texColor = textureCube(tCube, vec3(-1. * rayDir.x, rayDir.yz)); 
          gl_FragColor = vec4(texColor.rgb, 1.);
        }
      `
    })
    super(geometry, material)
    this.frustumCulled = false
    this.loadCubeMaps()
  }

  loadCubeMaps() {
    const loader = new CubeTextureLoader()
    const texNames = ['left', 'right', 'top', 'bottom', 'back', 'front']
    const imgs = texNames.map((name) => {
      return `./assets/skybox/${name}.png`
    })

    loader.load(imgs, (texture) => {
      this.material.uniforms.tCube.value = texture
    })
  }
}
