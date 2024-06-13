import { Mesh, ShaderMaterial, CubeTextureLoader, Matrix4, TextureLoader } from 'three'
import { FullScreenGeometry } from './full-screen-geometry.js'

export class BlackHole extends Mesh {
  constructor() {
    const geometry = new FullScreenGeometry()
    const material = new ShaderMaterial({
      transparent: true,
      uniforms: {
        skyBoxCube: { value: null },
        cameraWorldMat: { value: new Matrix4() },
        adiskParticle: { value: 1 },
        adiskHeight: { value: 0.2 },
        adiskLit: { value: 1 },
        adiskDensityV: { value: 1 },
        adiskDensityH: { value: 1 },
        adiskNoiseScale: { value: 0.8 },
        adiskNoiseLOD: { value: 5 },
        adiskSpeed: { value: 0.8 },
        colorMap: { value: null },
        iTime: { value: 0 },
        resolution: { value: 1 }
      },
      vertexShader: `
        varying vec2 vUv;
        varying vec3 rayDir;
        varying mat4 projectionMat;

        void main() {
          vUv = uv;
          projectionMat = projectionMatrix;
          gl_Position = vec4(position.xy, -1.0, 1.0);
        }
      `,
      fragmentShader: `
        varying vec2 vUv;
        uniform samplerCube skyBoxCube;
        varying mat4 projectionMat;
        uniform mat4 cameraWorldMat;

        uniform float adiskParticle;
        uniform float adiskHeight;
        uniform float adiskLit;
        uniform float adiskDensityV;
        uniform float adiskDensityH;
        uniform float adiskNoiseScale;
        uniform float adiskNoiseLOD;
        uniform float adiskSpeed;
        uniform sampler2D colorMap;
        uniform float iTime;
        uniform float resolution;


        vec4 quadConj(vec4 q) { return vec4(-q.x, -q.y, -q.z, q.w); }

        vec4 quadFromAxisAngle(vec3 axis, float angle) {
          vec4 qr;
          float half_angle = (angle * 0.5) * 3.14159 / 180.0;
          qr.x = axis.x * sin(half_angle);
          qr.y = axis.y * sin(half_angle);
          qr.z = axis.z * sin(half_angle);
          qr.w = cos(half_angle);
          return qr;
        }

        vec4 quat_mult(vec4 q1, vec4 q2) {
          vec4 qr;
          qr.x = (q1.w * q2.x) + (q1.x * q2.w) + (q1.y * q2.z) - (q1.z * q2.y);
          qr.y = (q1.w * q2.y) - (q1.x * q2.z) + (q1.y * q2.w) + (q1.z * q2.x);
          qr.z = (q1.w * q2.z) + (q1.x * q2.y) - (q1.y * q2.x) + (q1.z * q2.w);
          qr.w = (q1.w * q2.w) - (q1.x * q2.x) - (q1.y * q2.y) - (q1.z * q2.z);
          return qr;
        }

        vec3 rotateVector(vec3 position, vec3 axis, float angle) {
          vec4 qr = quadFromAxisAngle(axis, angle);
          vec4 qr_conj = quadConj(qr);
          vec4 q_pos = vec4(position.x, position.y, position.z, 0);
        
          vec4 q_tmp = quat_mult(qr, q_pos);
          qr = quat_mult(q_tmp, qr_conj);
        
          return vec3(qr.x, qr.y, qr.z);
        }

        vec3 unProjectPoint(vec3 pos) {
					vec4 p = cameraWorldMat * inverse(projectionMat) * vec4(pos.xyz, 1.0);
					return p.xyz / p.w;
				}

        vec4 texSkyBoxColor(vec3 dir) {
          return textureCube(skyBoxCube, vec3(-1.0 * dir.x, dir.yz)); 
        }

        vec3 accel(float h2, vec3 pos) {
          float r2 = dot(pos, pos);
          float r5 = pow(r2, 2.5);
          vec3 acc = -1.5 * h2 * pos / r5 * 1.0;
          return acc;
        }

        ///----
        /// Simplex 3D Noise
        /// by Ian McEwan, Ashima Arts
        vec4 permute(vec4 x) { return mod(((x * 34.0) + 1.0) * x, 289.0); }
        vec4 taylorInvSqrt(vec4 r) { return 1.79284291400159 - 0.85373472095314 * r; }

        float snoise(vec3 v) {
          const vec2 C = vec2(1.0 / 6.0, 1.0 / 3.0);
          const vec4 D = vec4(0.0, 0.5, 1.0, 2.0);

          // First corner
          vec3 i = floor(v + dot(v, C.yyy));
          vec3 x0 = v - i + dot(i, C.xxx);

          // Other corners
          vec3 g = step(x0.yzx, x0.xyz);
          vec3 l = 1.0 - g;
          vec3 i1 = min(g.xyz, l.zxy);
          vec3 i2 = max(g.xyz, l.zxy);

          //  x0 = x0 - 0. + 0.0 * C
          vec3 x1 = x0 - i1 + 1.0 * C.xxx;
          vec3 x2 = x0 - i2 + 2.0 * C.xxx;
          vec3 x3 = x0 - 1. + 3.0 * C.xxx;

          // Permutations
          i = mod(i, 289.0);
          vec4 p = permute(permute(permute(i.z + vec4(0.0, i1.z, i2.z, 1.0)) + i.y +
                                  vec4(0.0, i1.y, i2.y, 1.0)) +
                          i.x + vec4(0.0, i1.x, i2.x, 1.0));

          // Gradients
          // ( N*N points uniformly over a square, mapped onto an octahedron.)
          float n_ = 1.0 / 7.0; // N=7
          vec3 ns = n_ * D.wyz - D.xzx;

          vec4 j = p - 49.0 * floor(p * ns.z * ns.z); //  mod(p,N*N)

          vec4 x_ = floor(j * ns.z);
          vec4 y_ = floor(j - 7.0 * x_); // mod(j,N)

          vec4 x = x_ * ns.x + ns.yyyy;
          vec4 y = y_ * ns.x + ns.yyyy;
          vec4 h = 1.0 - abs(x) - abs(y);

          vec4 b0 = vec4(x.xy, y.xy);
          vec4 b1 = vec4(x.zw, y.zw);

          vec4 s0 = floor(b0) * 2.0 + 1.0;
          vec4 s1 = floor(b1) * 2.0 + 1.0;
          vec4 sh = -step(h, vec4(0.0));

          vec4 a0 = b0.xzyw + s0.xzyw * sh.xxyy;
          vec4 a1 = b1.xzyw + s1.xzyw * sh.zzww;

          vec3 p0 = vec3(a0.xy, h.x);
          vec3 p1 = vec3(a0.zw, h.y);
          vec3 p2 = vec3(a1.xy, h.z);
          vec3 p3 = vec3(a1.zw, h.w);

          // Normalise gradients
          vec4 norm =
              taylorInvSqrt(vec4(dot(p0, p0), dot(p1, p1), dot(p2, p2), dot(p3, p3)));
          p0 *= norm.x;
          p1 *= norm.y;
          p2 *= norm.z;
          p3 *= norm.w;

          // Mix final noise value
          vec4 m =
              max(0.6 - vec4(dot(x0, x0), dot(x1, x1), dot(x2, x2), dot(x3, x3)), 0.0);
          m = m * m;
          return 42.0 *
                dot(m * m, vec4(dot(p0, x0), dot(p1, x1), dot(p2, x2), dot(p3, x3)));
        }
        ///----

        vec3 toSpherical(vec3 p) {
          float rho = sqrt((p.x * p.x) + (p.y * p.y) + (p.z * p.z));
          float theta = atan(p.z, p.x);
          float phi = asin(p.y / rho);
          return vec3(rho, theta, phi);
        }

        void adiskColor(vec3 pos, inout vec3 color, inout float alpha) {
          float innerRadius = 2.6;
          float outerRadius = 10.0;
        
          // Density linearly decreases as the distance to the blackhole center
          // increases.
          float density = max(
              0.0, 1.0 - length(pos.xyz / vec3(outerRadius, adiskHeight, outerRadius)));
          if (density < 0.001) {
            return;
          }
        
          density *= pow(1.0 - abs(pos.y) / adiskHeight, adiskDensityV);
        
          // Set particale density to 0 when radius is below the inner most stable
          // circular orbit.
          density *= smoothstep(innerRadius, innerRadius * 1.1, length(pos));
        
          // Avoid the shader computation when density is very small.
          if (density < 0.001) {
            return;
          }
        
          vec3 sphericalCoord = toSpherical(pos);
        
          // Scale the rho and phi so that the particales appear to be at the correct
          // scale visually.
          sphericalCoord.y *= 2.0;
          sphericalCoord.z *= 4.0;
        
          density *= 1.0 / pow(sphericalCoord.x, adiskDensityH);
          density *= 16000.0;
        
          if (adiskParticle < 0.5) {
            color += vec3(0.0, 1.0, 0.0) * density * 0.02;
            return;
          }
        
          float noise = 1.0;
          for (int i = 0; i < int(adiskNoiseLOD); i++) {
            noise *= 0.5 * snoise(sphericalCoord * pow(float(i), 2.0) * adiskNoiseScale) + 0.5;
            if (i % 2 == 0) {
              sphericalCoord.y += iTime * adiskSpeed;
            } else {
              sphericalCoord.y -= iTime * adiskSpeed;
            }
          }
        
          vec3 dustColor =
            texture2D(colorMap, vec2(sphericalCoord.x / outerRadius, 0.5)).rgb;
        
          color += density * adiskLit * dustColor * alpha * abs(noise) * 0.004;
        }

        vec3 traceColor(vec3 pos, vec3 dir) {
          vec3 color = vec3(0);
          float alpha = 1.0;
          float STEP_SIZE = 0.1;
          dir *= STEP_SIZE;

          vec3 h = cross(pos, dir);
          float h2 = dot(h, h);

          for (int i = 0; i < 300; i++) {
            vec3 acc = accel(h2, pos);
            dir += acc;
      
            // Reach event horizon
            if (dot(pos, pos) < 1.0) {
              return color;
            }
            adiskColor(pos, color, alpha);
            pos += dir;
          }
          dir = rotateVector(dir, vec3(0.0, 1.0, 0.0), iTime);
          color += texSkyBoxColor(dir).rgb * alpha;
          return color;
        }


        ///----
        /// Narkowicz 2015, "ACES Filmic Tone Mapping Curve"
        vec3 aces(vec3 x) {
          const float a = 2.51;
          const float b = 0.03;
          const float c = 2.43;
          const float d = 0.59;
          const float e = 0.14;
          return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
        }

        float aces(float x) {
          const float a = 2.51;
          const float b = 0.03;
          const float c = 2.43;
          const float d = 0.59;
          const float e = 0.14;
          return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
        }
        ///----

        void main() {
          vec3 rayDir = vec3(0.0, 0.0, -1.0);
          vec2 uv = vec2(vUv);
          uv -= 0.5;
          uv /= 0.5;
          vec3 fragPos = unProjectPoint(vec3(uv, -1.0));
          rayDir = normalize(fragPos - cameraPosition);
          vec4 texBlackHoleColor = vec4(traceColor(cameraPosition, rayDir), 1.0);

          // tonemapping
          texBlackHoleColor.rgb = aces(texBlackHoleColor.rgb);
          texBlackHoleColor.rgb = pow(texBlackHoleColor.rgb, vec3(1.0 / 2.2));

          gl_FragColor = texBlackHoleColor;
        }
      `
    })
    super(geometry, material)
    this.loadMaps()

    this.onBeforeRender = (_, __, camera) => {
      material.uniforms.cameraWorldMat.value = camera.matrixWorld
    }
  }

  loadMaps() {
    const cubeLoader = new CubeTextureLoader()
    const textureLoader = new TextureLoader()
    const texNames = ['left', 'right', 'top', 'bottom', 'back', 'front']
    const imgs = texNames.map((name) => {
      return `./assets/skybox/${name}.png`
    })

    cubeLoader.load(imgs, (texture) => {
      this.material.uniforms.skyBoxCube.value = texture
    })

    textureLoader.load('./assets/color_map.png', (texture) => {
      this.material.uniforms.colorMap.value = texture
    })
  }

  update(elapsed) {
    this.material.uniforms.iTime.value = elapsed
  }
}
