//! OpenGL scene/HUD programs, mesh upload, and chase-camera math.

use crate::model;
use crate::physics;
use crate::render;
use crate::util;
use crate::util::geometry::XYZ;
#[cfg(test)]
use crate::util::geometry::XZ;
use crate::world;
use sdl3::video::Window;
use std::ffi::CString;

pub const VS_SRC: &str = "#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;
layout (location = 2) in vec2 uv;
uniform mat4 u_mvp;
out vec3 v_color;
out vec2 v_uv;
void main() { gl_Position = u_mvp * vec4(position, 1.0); v_color = color; v_uv = uv; }";

pub const FS_SRC: &str = "#version 330 core
out vec4 o;
in vec3 v_color;
in vec2 v_uv;
uniform vec3 u_tint;
uniform sampler2D u_glyphs;
void main() {
    float coverage = v_uv.x < 0.0 ? 1.0 : texture(u_glyphs, v_uv).r;
    o = vec4(v_color * u_tint, coverage);
}";

pub const SCENE_VS: &str = "#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;
layout (location = 2) in vec3 normal;
layout (location = 3) in vec2 material;
uniform mat4 u_clip_from_model;
uniform mat4 u_model;
out vec3 v_color;
out vec3 v_normal;
out vec3 v_world;
out vec2 v_material;
void main() {
    vec4 world = u_model * vec4(position, 1.0);
    v_world = world.xyz;
    v_normal = mat3(u_model) * normal;
    v_color = color;
    v_material = material;
    gl_Position = u_clip_from_model * vec4(position, 1.0);
}";

pub const SCENE_FS: &str = "#version 330 core
in vec3 v_color;
in vec3 v_normal;
in vec3 v_world;
in vec2 v_material;
uniform vec3 u_tint;
uniform vec3 u_eye;
uniform int u_contact_count;
uniform vec3 u_contact_origins[2];
uniform vec4 u_contact_axes[2];
uniform float u_contact_strength;
uniform vec2 u_fog_range;
uniform float u_signal_time;
uniform float u_brake_light;
uniform float u_reverse_light;
uniform int u_city_reference;
uniform int u_concrete;
uniform sampler2DArray u_aerial;
uniform int u_aerial_count;
uniform vec4 u_aerial_bounds[48];
uniform int u_aerial_layers[48];
uniform vec2 u_aerial_uv;
out vec4 o;
float surface_hash(vec2 p) {
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}
float surface_noise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(surface_hash(i), surface_hash(i + vec2(1,0)), f.x),
               mix(surface_hash(i + vec2(0,1)), surface_hash(i + vec2(1,1)), f.x), f.y);
}
float asphalt_mask() {
    if (v_material.x < -1.5 || v_material.x > -0.5 || normalize(v_normal).y < 0.65) return 0.0;
    float road = min(distance(v_color,vec3(0.23,0.25,0.26)),
        min(min(distance(v_color,vec3(0.18,0.20,0.22)),distance(v_color,vec3(0.25,0.26,0.26))),
        min(distance(v_color,vec3(0.32,0.31,0.28)),distance(v_color,vec3(0.42,0.43,0.40)))));
    return 1.0 - step(0.012, road);
}
vec3 asphalt_texture(vec3 color) {
    vec2 p = v_world.xz;
    // Irregular exposed aggregate in a darker bitumen binder, at centimetre scale.
    vec2 q = p * 55.0, cell = floor(q), local = fract(q);
    float nearest = 10.0, tone = 0.5;
    for (int y=-1; y<=1; ++y) for (int x=-1; x<=1; ++x) {
        vec2 offset = vec2(float(x),float(y));
        vec2 key = cell + offset;
        vec2 seed = vec2(surface_hash(key),surface_hash(key + vec2(37.0,91.0)));
        float d = length(offset + seed - local);
        if (d < nearest) { nearest=d; tone=surface_hash(key+vec2(13.0,7.0)); }
    }
    float aa = max(length(fwidth(q)),0.02);
    float stone = 1.0-smoothstep(0.20,0.36+aa,nearest);
    float detail = 1.0-smoothstep(0.35,1.6,length(fwidth(q)));
    float aggregate = mix(1.0,0.83+stone*(0.24+tone*0.25),detail);
    float pores = (surface_noise(p*90.0)-0.5)*0.08*detail;
    float weather = (surface_noise(p*0.45)-0.5)*0.10;
    // Soft, irregular resurfacing avoids a repeating grid of square patches.
    float repair = smoothstep(0.70,0.80,surface_noise(p*0.11));
    return color * (aggregate + pores + weather - repair*0.09);
}
// Baked map palettes select the surface, not colours sampled from the photo.
// Keep this pass on upward ground so walls, trees, paint and markings stay clean.
vec3 mapped_ground_detail(vec3 color) {
    if (v_material.x < -1.5 || v_material.x > -0.5 || normalize(v_normal).y < 0.8)
        return color;
    float grass = min(min(distance(v_color,vec3(0.39,0.48,0.25)),
                          distance(v_color,vec3(0.43,0.53,0.30))),
                      min(distance(v_color,vec3(0.40,0.46,0.28)),
                          min(distance(v_color,vec3(0.33,0.42,0.21)),
                              distance(v_color,vec3(0.28,0.38,0.19)))));
    float paving = min(min(distance(v_color,vec3(0.68,0.69,0.66)),
                           distance(v_color,vec3(0.72,0.71,0.65))),
                       distance(v_color,vec3(0.66,0.64,0.56)));
    float concrete = distance(v_color,vec3(0.72,0.73,0.70));
    float dirt = min(min(distance(v_color,vec3(0.62,0.58,0.46)),
                         distance(v_color,vec3(0.56,0.51,0.40))),
                     min(distance(v_color,vec3(0.64,0.63,0.54)),
                         distance(v_color,vec3(0.43,0.42,0.32))));
    if (min(min(grass,paving),min(concrete,dirt)) > 0.018) return color;
    vec2 p = v_world.xz;
    vec2 dx = dFdx(p), dy = dFdy(p);
    // Use pixel area for detail visibility. The longest axis grows rapidly on
    // ground viewed at a shallow angle and was hiding detail only metres away.
    float footprint = sqrt(abs(dx.x*dy.y-dx.y*dy.x));
    float fade = 1.0-smoothstep(25.0,65.0,length(u_eye-v_world));
    if (fade < 0.001) return color;
    float fine = 1.0-smoothstep(0.08,0.45,footprint);
    float aa = max(footprint*0.7,0.0015);
    if (grass < 0.018) {
        // Jittered, leaning strokes suggest short tufts without a regular stripe grid.
        vec2 q = mat2(0.917,0.399,-0.399,0.917)*p*vec2(7.0,5.0);
        vec2 cell = floor(q), local = fract(q);
        float seed = surface_hash(cell);
        float along = local.y-0.2;
        float center = 0.35+0.30*surface_hash(cell+31.0)
            +(seed-0.5)*along*0.65;
        float blade_aa = max(fwidth(local.x-center)*0.5,0.01);
        float blade = (1.0-smoothstep(0.035,0.085+blade_aa,abs(local.x-center)))
            * smoothstep(0.0,0.12,along)
            * (1.0-smoothstep(0.42,0.65,along));
        float clumps = (surface_noise(p*1.9)-0.5)*0.24
            +(surface_noise(p*5.0)-0.5)*0.12*fine;
        float strokes = blade*mix(-0.32,0.30,step(0.46,seed))*fine;
        return color*(1.0+fade*(clumps+strokes));
    }
    if (min(paving,concrete) < 0.018) {
        // Staggered pavers and larger concrete slabs. Dimensions are in metres.
        bool slabs = concrete < paving;
        vec2 size = slabs ? vec2(1.20,0.90) : vec2(0.60,0.40);
        vec2 q = mat2(0.917,0.399,-0.399,0.917)*p;
        float row = floor(q.y/size.y);
        q.x += slabs ? 0.0 : mod(row,2.0)*size.x*0.5;
        vec2 cell = floor(q/size);
        vec2 edge = min(fract(q/size),1.0-fract(q/size))*size;
        float gap_distance = min(edge.x,edge.y);
        float width = slabs ? 0.007 : 0.010;
        // Filter across each joint rather than along the entire ground footprint.
        float aa = max(fwidth(gap_distance)*0.5,0.0015);
        float joints = (1.0-smoothstep(max(0.0,width-aa),width+aa,gap_distance))
            * min(1.0,width/aa);
        float bevel = smoothstep(width,width+0.008,gap_distance)
            * (1.0-smoothstep(width+0.008,width+0.018+aa,gap_distance));
        float variation = (surface_hash(cell)-0.5)*0.12;
        float grain = (surface_noise(p*45.0)-0.5)*0.06*fine;
        float resolved = 1.0-smoothstep(0.15,0.80,footprint);
        return color*(1.0+fade*(variation+resolved*(grain-joints*0.40+bevel*0.07)));
    }
    // Loose earth: broad clods plus occasional small stones with a dark rim.
    float clods = (surface_noise(p*2.2)-0.5)*0.28
        +(surface_noise(p*8.0)-0.5)*0.16*fine;
    vec2 q = p/0.22, cell = floor(q), local = fract(q);
    vec2 center = vec2(0.3)+vec2(surface_hash(cell+11.0),surface_hash(cell+47.0))*0.4;
    vec2 delta = (local-center)*vec2(1.0,1.4)*0.22;
    float radius = mix(0.018,0.045,surface_hash(cell+73.0));
    float d = length(delta);
    aa = max(fwidth(d)*0.5,0.0015);
    float stone = 1.0-smoothstep(max(0.0,radius-aa),radius+aa,d);
    float rim = (1.0-smoothstep(radius,radius+0.012+aa,d))-stone;
    float scatter = step(0.68,surface_hash(cell+97.0));
    float relief = scatter*(stone*(0.18+0.18*surface_hash(cell+23.0))-rim*0.20)*fine;
    return color*(1.0+fade*(clods+relief));
}
vec3 concrete_texture(vec3 color, vec3 normal) {
    vec3 tangent = normalize(vec3(-normal.z, 0.0, normal.x));
    vec2 p = vec2(dot(v_world, tangent), v_world.y);
    // Keep authored plaster, limestone and trim colours distinct.
    float fine = 1.0 - smoothstep(0.35, 1.5, length(fwidth(p * 65.0)));
    float pores = surface_hash(floor(p * 65.0)) - 0.5;
    float weather = surface_noise(p * 0.7) - 0.5;
    return color * (1.0 + pores * 0.04 * fine + weather * 0.035);
}

float contact_visibility() {
    if (v_material.x > -0.5) return 1.0;
    float visibility = 1.0;
    for (int i = 0; i < u_contact_count; ++i) {
        vec2 delta = v_world.xz - u_contact_origins[i].xz;
        vec4 axes = u_contact_axes[i];
        vec2 local = vec2(dot(delta, vec2(axes.y, -axes.x)),
                          dot(delta, axes.xy)) / axes.zw;
        float soft = 1.0 - smoothstep(0.16, 1.0, dot(local, local));
        float height = 1.0 - smoothstep(0.3, 1.2,
            abs(v_world.y + 0.43 - u_contact_origins[i].y));
        visibility *= 1.0 - u_contact_strength * soft * height;
    }
    return visibility;
}
void main() {
    // Storefront lettering has one readable face. Its back must never show
    // mirrored text while a streamed facade is still loading.
    if (v_material.x < -2.5 && !gl_FrontFacing) discard;
    // Reserved material -2: emissive signal lenses. The alpha-free material
    // carries an integer program offset and lens index, with a fractional guard.
    if (v_material.x > -2.5 && v_material.x < -1.5) {
        float code = floor(v_material.y);
        float lens = mod(code, 4.0);
        float phase = mod(u_signal_time + floor(code / 4.0), 50.0);
        bool is_lit = (lens < 0.5 && phase >= 23.0)
            || (lens > 0.5 && lens < 1.5 && ((phase >= 20.0 && phase < 23.0) || phase >= 48.0))
            || (lens > 1.5 && phase < 20.0);
        vec3 lit = v_color * (is_lit ? 1.0 : 0.055);
        float fog = smoothstep(u_fog_range.x, u_fog_range.y, length(u_eye - v_world));
        o = vec4(mix(lit, vec3(0.57,0.73,0.80), fog), 1.0);
        return;
    }
    // World-space projection follows the existing raised surface meshes.
    if (v_material.x > -1.5 && v_material.x < -0.5
        && normalize(v_normal).y > 0.8 && v_color.r < 0.84) {
        for (int i=0;i<u_aerial_count;++i) {
            vec4 bounds=u_aerial_bounds[i];
            vec2 uv=(v_world.xz-bounds.xy)/(bounds.zw-bounds.xy);
            if (all(greaterThanEqual(uv,vec2(0.0))) && all(lessThan(uv,vec2(1.0)))) {
                vec3 colour=texture(u_aerial,vec3(vec2(u_aerial_uv.x)+uv*u_aerial_uv.y,float(u_aerial_layers[i]))).rgb;
                if (asphalt_mask() > 0.5) colour *= 0.55;
                else colour = mapped_ground_detail(colour);
                float fog=smoothstep(u_fog_range.x,u_fog_range.y,length(u_eye-v_world));
                o=vec4(mix(colour*contact_visibility(),vec3(0.57,0.73,0.80),fog),1.0);
                return;
            }
        }
    }
    vec3 n = normalize(v_normal);
    if (!gl_FrontFacing) n = -n;
    vec3 sun = normalize(vec3(-0.45, 0.85, -0.3));
    vec3 view = normalize(u_eye - v_world);
    float paint_mask = step(0.5, v_material.x) * (1.0 - step(1.5, v_material.x));
    vec3 paint = v_color * mix(vec3(1.0), u_tint, paint_mask);
    if (asphalt_mask() > 0.5) paint = asphalt_texture(paint);
    else paint = mapped_ground_detail(paint);
    if (u_concrete == 1 && abs(n.y) < 0.35 && abs(v_material.x) < 0.01
        && v_material.y < 0.05 && min(v_color.r, min(v_color.g, v_color.b)) > 0.55) {
        paint = concrete_texture(paint, n);
    }
    vec3 ambient = mix(vec3(0.25, 0.27, 0.23), vec3(0.53, 0.61, 0.67), n.y * 0.5 + 0.5);
    // Bright Mediterranean sky and pale ground bounce keep shaded Nicosia
    // walls readable. Car finishes have their own lighting below.
    if (u_concrete == 1 && v_material.x < 0.5) {
        ambient = mix(vec3(0.48, 0.49, 0.45), vec3(0.85, 0.90, 0.95), n.y * 0.5 + 0.5);
    }
    float diffuse = max(dot(n, sun), 0.0);
    float highlight = pow(max(dot(n, normalize(sun + view)), 0.0), 48.0)
        * v_material.y * step(0.001, diffuse);
    // Reference-block surface study: world-space grain and paving joints.
    // Keep the pilot local; no image textures or per-frame generated assets.
    if (u_city_reference == 1 && v_material.x > -1.5 && v_material.x < -0.5
        && abs(v_world.x) < 140.0 && abs(v_world.z) < 140.0) {
        vec2 tile = floor(v_world.xz * 80.0);
        float grain = fract(sin(dot(tile, vec2(127.1, 311.7))) * 43758.5453);
        if (v_color.r < 0.5 && abs(v_color.r - v_color.g) < 0.06) {
            float fine = 1.0 - smoothstep(0.4, 1.5, max(fwidth(v_world.x * 80.0), fwidth(v_world.z * 80.0)));
            paint *= 1.0 + (grain - 0.5) * 0.06 * fine;
        } else if (v_color.r > 0.65 && v_color.r < 0.82
            && abs(v_color.r - v_color.g) < 0.035) {
            vec2 joint = abs(fract(v_world.xz / 0.6) - 0.5);
            float gap = max(smoothstep(0.475, 0.495, joint.x),
                            smoothstep(0.475, 0.495, joint.y));
            paint *= (0.98 + grain * 0.04) * (1.0 - gap * 0.16);
        }
    }
    vec3 lit = paint * (ambient + vec3(0.65, 0.59, 0.46) * diffuse)
        + vec3(0.8, 0.77, 0.66) * highlight;
    // A broad sky/ground reflection makes curved paint and glass readable
    // between sun highlights. Matte road and terrain receive none.
    vec3 reflected = reflect(-view, n);
    vec3 environment = mix(vec3(0.18, 0.20, 0.16), vec3(0.57, 0.73, 0.80),
        smoothstep(-0.15, 0.55, reflected.y));
    float fresnel = 0.04 + 0.56 * pow(1.0 - max(dot(n, view), 0.0), 5.0);
    lit = mix(lit, environment, fresnel * clamp(v_material.y, 0.0, 1.0));
    // Authored car finishes: linear-light paint with a dielectric clear coat,
    // reflective glazing, polished metal, and coloured optical lenses. The
    // analytic daylight environment is stable as the car and camera turn.
    // Untagged world surfaces and older fixed-colour mesh details keep their
    // existing lighting; these tags require no scene archive format change.
    if (v_material.x > 0.5) {
        float gloss = clamp(v_material.y, 0.0, 1.0);
        float nv = max(dot(n, view), 0.0);
        float grazing = pow(1.0 - nv, 5.0);
        float sky = smoothstep(-0.06, 0.12, reflected.y);
        vec3 ground_reflection = mix(vec3(0.065, 0.075, 0.055),
            vec3(0.22, 0.24, 0.22), smoothstep(-0.65, 0.02, reflected.y));
        vec3 sky_reflection = mix(vec3(0.78, 0.85, 0.88), vec3(0.30, 0.49, 0.69),
            smoothstep(0.02, 0.85, reflected.y));
        float cloud = exp(-pow((reflected.y - 0.38 - 0.10 * reflected.x) / 0.12, 2.0));
        sky_reflection = mix(sky_reflection, vec3(0.91, 0.93, 0.92), cloud * 0.45);
        vec3 env = mix(ground_reflection, sky_reflection, sky);
        float nh = max(dot(n, normalize(sun + view)), 0.0);
        float glint = pow(nh, mix(48.0, 220.0, gloss)) * step(0.001, diffuse);
        vec3 base = pow(max(paint, vec3(0.0)), vec3(2.2));
        vec3 irradiance = mix(vec3(0.30, 0.32, 0.29), vec3(0.68, 0.75, 0.82), n.y * 0.5 + 0.5)
            + vec3(0.70, 0.64, 0.53) * diffuse;
        vec3 finish;
        if (v_material.x < 1.5) {
            float coat = (0.035 + 0.48 * grazing) * gloss;
            finish = base * irradiance * (1.0 - coat) + env * coat
                + vec3(1.0, 0.95, 0.84) * glint * 0.55 * gloss
                + base * pow(nh, 24.0) * 0.12 * gloss;
        } else if (v_material.x < 2.5) {
            finish = base * 0.45 + env * (0.10 + 0.64 * grazing) * gloss
                + vec3(0.85, 0.92, 1.0) * glint * 0.65;
        } else if (v_material.x < 3.5) {
            finish = base * (env * (0.65 + 0.25 * gloss) + irradiance * 0.13)
                + vec3(1.0, 0.96, 0.88) * glint * gloss;
        } else {
            finish = base * (irradiance * 0.72 + vec3(0.18))
                + env * (0.025 + 0.16 * grazing) + vec3(1.0) * glint * 0.4;
        }
        lit = pow(clamp(finish, 0.0, 1.0), vec3(1.0 / 2.2));
    }
    if (v_material.x > 4.5 && v_material.x < 5.5) {
        // Emission remains visible in shade and ignores the selected paint.
        lit = mix(lit, vec3(1.0, 0.035, 0.012), u_brake_light);
    }
    if (v_material.x > 5.5 && v_material.x < 6.5) {
        lit = mix(lit, vec3(1.0, 1.0, 0.94), u_reverse_light);
    }
    // Evaluate on the receiving road itself: no floating decal or depth bias.
    lit *= contact_visibility();
    float fog = smoothstep(u_fog_range.x, u_fog_range.y, length(u_eye - v_world));
    o = vec4(mix(lit, vec3(0.57, 0.73, 0.80), fog), 1.0);
}";

const PARTICLE_VS: &str = "#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec2 uv;
layout (location = 2) in vec3 color;
layout (location = 3) in float alpha;
uniform mat4 u_view_projection;
out vec3 v_world;
out vec2 v_uv;
out vec3 v_color;
out float v_alpha;
void main() {
    v_world = position;
    v_uv = uv;
    v_color = color;
    v_alpha = alpha;
    gl_Position = u_view_projection * vec4(position, 1.0);
}";

const PARTICLE_FS: &str = "#version 330 core
in vec3 v_world;
in vec2 v_uv;
in vec3 v_color;
in float v_alpha;
uniform vec3 u_eye;
uniform vec2 u_fog_range;
out vec4 o;
void main() {
    float radius = dot(v_uv, v_uv);
    if (radius >= 1.0) discard;
    float soft = (1.0 - radius) * (1.0 - radius);
    float fog = smoothstep(u_fog_range.x, u_fog_range.y, length(u_eye - v_world));
    vec3 color = mix(v_color, vec3(0.57, 0.73, 0.80), fog);
    o = vec4(color, v_alpha * soft * (1.0 - fog));
}";

const GL_VERTEX_SHADER: u32 = 0x8B31;
const GL_FRAGMENT_SHADER: u32 = 0x8B30;
const GL_COMPILE_STATUS: u32 = 0x8B81;
const GL_LINK_STATUS: u32 = 0x8B82;
const GL_INFO_LOG_LENGTH: u32 = 0x8B84;
const GL_ARRAY_BUFFER: u32 = 0x8892;
const GL_STATIC_DRAW: u32 = 0x88E4;
const GL_FLOAT: u32 = 0x1406;
const GL_TRIANGLES: u32 = 0x0004;
const GL_DEPTH_TEST: u32 = 0x0B71;
const GL_FALSE: u8 = 0;
const FOG_RANGE: [f32; 2] = [80.0, 340.0];
const VIEW_DISTANCE: f32 = 440.0;

struct Shader {
    handle: gl::types::GLuint,
}

fn compile(src: &str, kind: u32) -> Result<Shader, String> {
    let handle = unsafe { gl::CreateShader(kind) };
    let c_src = CString::new(src).map_err(|_| "NUL in shader source".to_string())?;
    let strings = [c_src.as_ptr()];
    let lengths = [c_src.count_bytes() as i32];
    unsafe {
        gl::ShaderSource(handle, 1, strings.as_ptr(), lengths.as_ptr());
        gl::CompileShader(handle);
        let mut status: i32 = 0;
        gl::GetShaderiv(handle, GL_COMPILE_STATUS, &mut status);
        if status != 1 {
            let mut log_len: i32 = 0;
            gl::GetShaderiv(handle, GL_INFO_LOG_LENGTH, &mut log_len);
            let mut log_buf: Vec<u8> = vec![0u8; log_len.max(1) as usize];
            gl::GetShaderInfoLog(
                handle,
                log_buf.len() as i32,
                &mut log_len,
                log_buf.as_mut_ptr() as *mut i8,
            );
            gl::DeleteShader(handle);
            let log = std::str::from_utf8(&log_buf).unwrap_or("shader info log not utf8");
            return Err(format!(
                "shader compilation failed: status={} info_log_len={} log={}",
                status,
                log_len,
                log.trim_end_matches('\0')
            ));
        }
    }
    Ok(Shader { handle })
}

impl Drop for Shader {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteShader(self.handle);
        }
    }
}

pub fn create_program(vs_src: &str, fs_src: &str) -> Result<u32, String> {
    let vs = compile(vs_src, GL_VERTEX_SHADER)?;
    let fs = compile(fs_src, GL_FRAGMENT_SHADER)?;
    unsafe {
        let program = gl::CreateProgram();
        gl::AttachShader(program, vs.handle);
        gl::AttachShader(program, fs.handle);
        gl::LinkProgram(program);
        let mut status: i32 = 0;
        gl::GetProgramiv(program, GL_LINK_STATUS, &mut status);
        if status != 1 {
            let mut log_len: i32 = 0;
            gl::GetProgramiv(program, GL_INFO_LOG_LENGTH, &mut log_len);
            let mut log_buf: Vec<u8> = vec![0u8; log_len.max(1) as usize];
            gl::GetProgramInfoLog(
                program,
                log_buf.len() as i32,
                &mut log_len,
                log_buf.as_mut_ptr() as *mut i8,
            );
            gl::DeleteProgram(program);
            let log = std::str::from_utf8(&log_buf).unwrap_or("program info log not utf8");
            return Err(format!(
                "program link failed: status={} info_log_len={} log={}",
                status,
                log_len,
                log.trim_end_matches('\0')
            ));
        }
        gl::UseProgram(program);
        Ok(program)
    }
}

struct CityBuffer {
    vao: u32,
    vbo: u32,
    count: i32,
}

fn upload(vertices: &[render::scene::Vertex]) -> CityBuffer {
    unsafe {
        let mut vao = 0;
        let mut vbo = 0;
        gl::GenVertexArrays(1, &mut vao);
        gl::GenBuffers(1, &mut vbo);
        gl::BindVertexArray(vao);
        gl::BindBuffer(GL_ARRAY_BUFFER, vbo);
        gl::BufferData(
            GL_ARRAY_BUFFER,
            std::mem::size_of_val(vertices) as isize,
            vertices.as_ptr().cast(),
            GL_STATIC_DRAW,
        );
        scene_attributes();
        CityBuffer {
            vao,
            vbo,
            count: vertices.len() as i32,
        }
    }
}

impl Drop for CityBuffer {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteBuffers(1, &self.vbo);
            gl::DeleteVertexArrays(1, &self.vao);
        }
    }
}
struct CityGpu {
    ground: CityBuffer,
    buildings: CityBuffer,
    coarse: bool,
}
struct CityChunk {
    entry: render::scene_archive::Entry,
    gpu: Option<CityGpu>,
}
fn coarse_lod(previous: Option<bool>, distance_squared: f32) -> bool {
    let distance = if previous == Some(true) {
        160.0_f32
    } else if previous == Some(false) {
        200.0
    } else {
        180.0
    };
    distance_squared > distance * distance
}

pub struct Mesh {
    vao: u32,
    vbo: u32,
    ground_vertex_count: i32,
    city_chunks: std::cell::RefCell<Vec<CityChunk>>,
    archive: std::cell::RefCell<Option<render::scene_archive::Archive>>,
    prefetch: Option<render::prefetch::Prefetch>,
    aerial: std::cell::RefCell<Option<render::aerial::Aerial>>,
    visible_chunks: std::cell::Cell<usize>,
    car_ranges: Vec<std::ops::Range<i32>>,
    wheel_range: Vec<[std::ops::Range<i32>; 4]>,
    concrete_region: bool,
}

impl Drop for Mesh {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteBuffers(1, &self.vbo);
            gl::DeleteVertexArrays(1, &self.vao);
        }
    }
}

pub fn mesh_new(
    track: &world::locations::WorldLocation,
    cars: &[std::sync::Arc<model::car::CarSpec>],
) -> Result<Mesh, String> {
    let archive = if world::street_surface::has_compiled_roads(track) {
        Some(
            render::scene_archive::open_for(
                model::world::assets(track).ok_or("compiled world has no asset directory")?,
            )
            .map_err(|e| e.to_string())?,
        )
    } else {
        None
    };
    let prefetch = archive
        .as_ref()
        .map(|a| render::scene_archive::fork(a).map(render::prefetch::new))
        .transpose()
        .map_err(|e| e.to_string())?;
    let city_chunks = archive
        .as_ref()
        .map(|a| {
            a.entries
                .iter()
                .map(|entry| CityChunk {
                    entry: *entry,
                    gpu: None,
                })
                .collect()
        })
        .unwrap_or_default();
    let (vao, vbo, ground_vertex_count, car_ranges, wheel_range) = unsafe {
        let mut vao: u32 = 0;
        let mut vbo: u32 = 0;
        gl::GenVertexArrays(1, &mut vao);
        gl::GenBuffers(1, &mut vbo);
        gl::BindVertexArray(vao);
        gl::BindBuffer(GL_ARRAY_BUFFER, vbo);
        let render::scene::Geometry {
            vertices: verts,
            world_count: ground_vertex_count,
            car_ranges,
            wheel_range,
        } = render::scene::build(track, cars);
        gl::BufferData(
            GL_ARRAY_BUFFER,
            (verts.len() * std::mem::size_of::<render::scene::Vertex>()) as isize,
            verts.as_ptr() as *const std::ffi::c_void,
            GL_STATIC_DRAW,
        );
        scene_attributes();
        gl::BindVertexArray(0);
        (
            vao,
            vbo,
            ground_vertex_count as i32,
            car_ranges,
            wheel_range,
        )
    };
    let concrete_region = track.origin.is_some_and(|origin| {
        (35.10..35.24).contains(&origin.lat) && (33.24..33.48).contains(&origin.lon)
    });
    Ok(Mesh {
        concrete_region,
        vao,
        vbo,
        ground_vertex_count,
        city_chunks: std::cell::RefCell::new(city_chunks),
        archive: std::cell::RefCell::new(archive),
        prefetch,
        aerial: std::cell::RefCell::new(if world::street_surface::has_compiled_roads(track) {
            model::world::assets(track).and_then(render::aerial::open)
        } else {
            None
        }),
        visible_chunks: std::cell::Cell::new(0),
        car_ranges,
        wheel_range,
    })
}

pub fn mesh_draw(
    mesh: &Mesh,
    program: u32,
    camera: &Camera,
    model: &[f32; 16],
    tint: [f32; 3],
    car: &physics::car::Vehicle,
    car_index: usize,
) {
    unsafe {
        gl::BindVertexArray(mesh.vao);
        gl::UseProgram(program);
        set_scene_transform(program, camera, model);
        set_tint(program, tint);
        let stone = CString::new("u_concrete").unwrap();
        gl::Uniform1i(gl::GetUniformLocation(program, stone.as_ptr()), 0);
        let brake_light = CString::new("u_brake_light").unwrap();
        let brake_light = gl::GetUniformLocation(program, brake_light.as_ptr());
        gl::Uniform1f(brake_light, if car.brake_lights { 1.0 } else { 0.0 });
        let reverse_light = CString::new("u_reverse_light").unwrap();
        let reverse_light = gl::GetUniformLocation(program, reverse_light.as_ptr());
        gl::Uniform1f(reverse_light, if car.gear == 0 { 1.0 } else { 0.0 });
        let range = &mesh.car_ranges[car_index];
        gl::DrawArrays(GL_TRIANGLES, range.start, range.end - range.start);
        gl::Uniform1f(brake_light, 0.0);
        gl::Uniform1f(reverse_light, 0.0);
        set_tint(program, [1.0; 3]);
        for index in 0..4 {
            let local = vehicle_wheel_local_matrix(car, *model, index);
            let matrix = multiply(*model, local);
            let clip = multiply(clip_from_model(camera, model), local);
            set_scene_transform_clip(program, camera, &matrix, &clip);
            gl::DrawArrays(
                GL_TRIANGLES,
                mesh.wheel_range[car_index][index].start,
                mesh.wheel_range[car_index][index].end - mesh.wheel_range[car_index][index].start,
            );
        }
    }
}

fn plan_prefetch(mesh: &Mesh, camera: &Camera) {
    let Some(prefetch) = &mesh.prefetch else {
        return;
    };
    let chunks = mesh.city_chunks.borrow();
    let mut wanted = Vec::new();
    for (index, chunk) in chunks.iter().enumerate() {
        let distance = render::chunks::distance_squared(chunk.entry.bounds, camera.eye);
        if distance > (VIEW_DISTANCE + 128.0).powi(2) {
            continue;
        }
        let visible = render::chunks::visible(chunk.entry.bounds, camera.view_projection);
        let coarse = coarse_lod(chunk.gpu.as_ref().map(|g| g.coarse), distance);
        let rank = if visible { 0 } else { 2 };
        if let Some(gpu) = &chunk.gpu {
            if gpu.coarse != coarse {
                wanted.push((0, distance, (index, if coarse { 2 } else { 1 })));
            }
        } else {
            wanted.push((rank, distance, (index, 0)));
            wanted.push((rank, distance, (index, if coarse { 2 } else { 1 })));
        }
        if let Some(gpu) = &chunk.gpu {
            if gpu.coarse && distance < 240.0_f32.powi(2) {
                wanted.push((1, distance, (index, 1)));
            }
            if !gpu.coarse && distance > 150.0_f32.powi(2) {
                wanted.push((1, distance, (index, 2)));
            }
        }
    }
    wanted.sort_by(|a, b| a.0.cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)));
    render::prefetch::plan(prefetch, wanted.into_iter().map(|v| v.2).collect());
}
fn city_layer(
    mesh: &Mesh,
    archive: &mut render::scene_archive::Archive,
    index: usize,
    layer: usize,
) -> Vec<render::scene::Vertex> {
    if let Some(result) = mesh
        .prefetch
        .as_ref()
        .and_then(|p| render::prefetch::take(p, (index, layer)))
    {
        return result.expect("prefetch city geometry");
    }
    render::scene_archive::read_layer(archive, index, layer).expect("read city geometry")
}
pub fn draw_ground(
    mesh: &Mesh,
    program: u32,
    camera: &Camera,
    contacts: &[render::shadows::ContactShadow],
) {
    plan_prefetch(mesh, camera);
    unsafe {
        gl::BindVertexArray(mesh.vao);
        gl::UseProgram(program);
        let aerial_count = CString::new("u_aerial_count").unwrap();
        gl::Uniform1i(gl::GetUniformLocation(program, aerial_count.as_ptr()), 0);
        if let Some(aerial) = mesh.aerial.borrow_mut().as_mut() {
            render::aerial::bind(aerial, program, camera.eye);
        }
        let stone = CString::new("u_concrete").unwrap();
        gl::Uniform1i(
            gl::GetUniformLocation(program, stone.as_ptr()),
            mesh.concrete_region as i32,
        );
        let name = CString::new("u_city_reference").unwrap();
        gl::Uniform1i(
            gl::GetUniformLocation(program, name.as_ptr()),
            mesh.prefetch.is_some() as i32,
        );
        set_scene_transform(
            program,
            camera,
            &model_matrix(
                XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                0.0,
                0.0,
            ),
        );
        set_tint(program, [1.0; 3]);
        set_contacts(program, contacts);
        gl::DrawArrays(GL_TRIANGLES, 0, mesh.ground_vertex_count);
        let mut visible = 0;
        let mut archive = mesh.archive.borrow_mut();
        for (index, chunk) in mesh.city_chunks.borrow_mut().iter_mut().enumerate() {
            if render::chunks::distance_squared(chunk.entry.bounds, camera.eye)
                > (VIEW_DISTANCE + render::chunks::SIZE).powi(2)
            {
                chunk.gpu = None;
                continue;
            }
            if !render::chunks::visible(chunk.entry.bounds, camera.view_projection) {
                continue;
            }
            let coarse = coarse_lod(
                chunk.gpu.as_ref().map(|g| g.coarse),
                render::chunks::distance_squared(chunk.entry.bounds, camera.eye),
            );
            let archive = archive.as_mut().expect("city archive");
            if chunk.gpu.is_none() {
                let ground = {
                    let vertices = city_layer(mesh, archive, index, 0);
                    upload(&vertices)
                };
                let buildings = {
                    let vertices = city_layer(mesh, archive, index, if coarse { 2 } else { 1 });
                    upload(&vertices)
                };
                chunk.gpu = Some(CityGpu {
                    ground,
                    buildings,
                    coarse,
                });
            }
            let gpu = chunk.gpu.as_mut().unwrap();
            if gpu.coarse != coarse {
                let vertices = city_layer(mesh, archive, index, if coarse { 2 } else { 1 });
                gpu.buildings = upload(&vertices);
                gpu.coarse = coarse;
            }
            for buffer in [&gpu.ground, &gpu.buildings] {
                gl::BindVertexArray(buffer.vao);
                gl::DrawArrays(GL_TRIANGLES, 0, buffer.count);
            }
            visible += 1;
        }
        mesh.visible_chunks.set(visible);
    }
}

unsafe fn set_contacts(program: u32, contacts: &[render::shadows::ContactShadow]) {
    use crate::render::shadows::MAX_CONTACTS;
    use crate::render::shadows::STRENGTH;
    let count = contacts.len().min(MAX_CONTACTS);
    let mut origins = [[0.0; 3]; MAX_CONTACTS];
    let mut axes = [[0.0; 4]; MAX_CONTACTS];
    for (i, contact) in contacts.iter().take(count).enumerate() {
        origins[i] = (contact.origin).into();
        axes[i] = contact.axes;
    }
    let location = |name: &str| {
        let name = CString::new(name).unwrap();
        gl::GetUniformLocation(program, name.as_ptr())
    };
    gl::Uniform1i(location("u_contact_count"), count as i32);
    gl::Uniform1f(location("u_contact_strength"), STRENGTH);
    if count > 0 {
        gl::Uniform3fv(
            location("u_contact_origins[0]"),
            count as i32,
            origins.as_ptr() as *const f32,
        );
        gl::Uniform4fv(
            location("u_contact_axes[0]"),
            count as i32,
            axes.as_ptr() as *const f32,
        );
    }
}

unsafe fn scene_attributes() {
    // With a VBO bound, the final argument is a byte offset, not a Rust pointer.
    let stride = std::mem::size_of::<render::scene::Vertex>() as i32;
    for (attribute, size, offset) in [(0, 3, 0), (1, 3, 3), (2, 3, 6), (3, 2, 9)] {
        gl::EnableVertexAttribArray(attribute);
        gl::VertexAttribPointer(
            attribute,
            size,
            GL_FLOAT,
            GL_FALSE,
            stride,
            (offset * std::mem::size_of::<f32>()) as *const std::ffi::c_void,
        );
    }
}

pub struct SkidRenderer {
    vao: u32,
    vbo: u32,
}

pub fn skid_renderer_new() -> SkidRenderer {
    unsafe {
        let (mut vao, mut vbo) = (0, 0);
        gl::GenVertexArrays(1, &mut vao);
        gl::GenBuffers(1, &mut vbo);
        gl::BindVertexArray(vao);
        gl::BindBuffer(GL_ARRAY_BUFFER, vbo);
        scene_attributes();
        gl::BindVertexArray(0);
        SkidRenderer { vao, vbo }
    }
}

pub fn skid_renderer_draw(
    skid_renderer: &SkidRenderer,
    program: u32,
    camera: &Camera,
    marks: &render::skid_marks::SkidMarks,
) {
    let vertices = render::skid_marks::vertices(marks);
    if vertices.is_empty() {
        return;
    }
    unsafe {
        gl::UseProgram(program);
        gl::BindVertexArray(skid_renderer.vao);
        gl::BindBuffer(GL_ARRAY_BUFFER, skid_renderer.vbo);
        gl::BufferData(
            GL_ARRAY_BUFFER,
            std::mem::size_of_val(vertices.as_slice()) as isize,
            vertices.as_ptr() as *const _,
            gl::STREAM_DRAW,
        );
        set_scene_transform(
            program,
            camera,
            &model_matrix(
                XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                0.0,
                0.0,
            ),
        );
        set_tint(program, [1.0; 3]);
        gl::DepthMask(gl::FALSE);
        gl::DrawArrays(GL_TRIANGLES, 0, vertices.len() as i32);
        gl::DepthMask(gl::TRUE);
    }
}

impl Drop for SkidRenderer {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteBuffers(1, &self.vbo);
            gl::DeleteVertexArrays(1, &self.vao);
        }
    }
}

pub struct ParticleRenderer {
    vao: u32,
    vbo: u32,
    program: u32,
}

pub fn particle_renderer_new() -> Result<ParticleRenderer, String> {
    let program = create_program(PARTICLE_VS, PARTICLE_FS)?;
    unsafe {
        let (mut vao, mut vbo) = (0, 0);
        gl::GenVertexArrays(1, &mut vao);
        gl::GenBuffers(1, &mut vbo);
        gl::BindVertexArray(vao);
        gl::BindBuffer(GL_ARRAY_BUFFER, vbo);
        let stride = std::mem::size_of::<render::tire_particles::Vertex>() as i32;
        for (attribute, size, offset) in [(0, 3, 0), (1, 2, 3), (2, 3, 5), (3, 1, 8)] {
            gl::EnableVertexAttribArray(attribute);
            gl::VertexAttribPointer(
                attribute,
                size,
                GL_FLOAT,
                GL_FALSE,
                stride,
                (offset * std::mem::size_of::<f32>()) as *const std::ffi::c_void,
            );
        }
        gl::BindVertexArray(0);
        Ok(ParticleRenderer { vao, vbo, program })
    }
}

pub fn particle_renderer_draw(
    particle_renderer: &ParticleRenderer,
    camera: &Camera,
    particles: &render::tire_particles::TireParticles,
) -> usize {
    let vertices = render::tire_particles::vertices(particles, camera.particle_view);
    if vertices.is_empty() {
        return 0;
    }
    unsafe {
        let was_blend = gl::IsEnabled(gl::BLEND);
        let was_depth_test = gl::IsEnabled(gl::DEPTH_TEST);
        let mut depth_write = gl::TRUE;
        gl::GetBooleanv(gl::DEPTH_WRITEMASK, &mut depth_write);
        let mut blend = [0; 6];
        for (i, parameter) in [
            gl::BLEND_SRC_RGB,
            gl::BLEND_DST_RGB,
            gl::BLEND_SRC_ALPHA,
            gl::BLEND_DST_ALPHA,
            gl::BLEND_EQUATION_RGB,
            gl::BLEND_EQUATION_ALPHA,
        ]
        .into_iter()
        .enumerate()
        {
            gl::GetIntegerv(parameter, &mut blend[i]);
        }
        gl::UseProgram(particle_renderer.program);
        gl::BindVertexArray(particle_renderer.vao);
        gl::BindBuffer(GL_ARRAY_BUFFER, particle_renderer.vbo);
        gl::BufferData(
            GL_ARRAY_BUFFER,
            std::mem::size_of_val(vertices.as_slice()) as isize,
            vertices.as_ptr() as *const _,
            gl::STREAM_DRAW,
        );
        let name = CString::new("u_view_projection").unwrap();
        gl::UniformMatrix4fv(
            gl::GetUniformLocation(particle_renderer.program, name.as_ptr()),
            1,
            GL_FALSE,
            camera.view_projection.as_ptr(),
        );
        let name = CString::new("u_eye").unwrap();
        gl::Uniform3f(
            gl::GetUniformLocation(particle_renderer.program, name.as_ptr()),
            camera.eye.x,
            camera.eye.y,
            camera.eye.z,
        );
        set_fog(particle_renderer.program);
        gl::Enable(gl::DEPTH_TEST);
        gl::Enable(gl::BLEND);
        gl::BlendEquation(gl::FUNC_ADD);
        gl::BlendFunc(gl::SRC_ALPHA, gl::ONE_MINUS_SRC_ALPHA);
        gl::DepthMask(gl::FALSE);
        gl::DrawArrays(GL_TRIANGLES, 0, vertices.len() as i32);
        gl::DepthMask(depth_write);
        gl::BlendFuncSeparate(
            blend[0] as u32,
            blend[1] as u32,
            blend[2] as u32,
            blend[3] as u32,
        );
        gl::BlendEquationSeparate(blend[4] as u32, blend[5] as u32);
        if was_blend == gl::FALSE {
            gl::Disable(gl::BLEND);
        }
        if was_depth_test == gl::FALSE {
            gl::Disable(gl::DEPTH_TEST);
        }
    }
    vertices.len()
}

impl Drop for ParticleRenderer {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteProgram(self.program);
            gl::DeleteBuffers(1, &self.vbo);
            gl::DeleteVertexArrays(1, &self.vao);
        }
    }
}

unsafe fn set_scene_transform(program: u32, camera: &Camera, model: &[f32; 16]) {
    set_scene_transform_clip(program, camera, model, &clip_from_model(camera, model));
}

unsafe fn set_scene_transform_clip(
    program: u32,
    camera: &Camera,
    model: &[f32; 16],
    clip: &[f32; 16],
) {
    for (name, matrix) in [("u_clip_from_model", clip), ("u_model", model)] {
        let name = CString::new(name).unwrap();
        gl::UniformMatrix4fv(
            gl::GetUniformLocation(program, name.as_ptr()),
            1,
            GL_FALSE,
            matrix.as_ptr(),
        );
    }
    let name = CString::new("u_eye").unwrap();
    gl::Uniform3f(
        gl::GetUniformLocation(program, name.as_ptr()),
        camera.eye.x,
        camera.eye.y,
        camera.eye.z,
    );
    set_fog(program);
}

unsafe fn set_fog(program: u32) {
    let name = CString::new("u_fog_range").unwrap();
    gl::Uniform2f(
        gl::GetUniformLocation(program, name.as_ptr()),
        FOG_RANGE[0],
        FOG_RANGE[1],
    );
}

unsafe fn set_tint(program: u32, tint: [f32; 3]) {
    let name = CString::new("u_tint").unwrap();
    gl::Uniform3f(
        gl::GetUniformLocation(program, name.as_ptr()),
        tint[0],
        tint[1],
        tint[2],
    );
}

fn perspective(aspect: f32, fov_y: f32, near: f32, far: f32) -> [f32; 16] {
    let f = 1.0 / (fov_y * 0.5).tan();
    [
        f / aspect,
        0.0,
        0.0,
        0.0,
        0.0,
        f,
        0.0,
        0.0,
        0.0,
        0.0,
        (far + near) / (near - far),
        -1.0,
        0.0,
        0.0,
        (2.0 * far * near) / (near - far),
        0.0,
    ]
}

pub struct Camera {
    view_projection: [f32; 16],
    relative_view_projection: [f32; 16],
    eye: XYZ,
    particle_view: render::tire_particles::View,
}

fn clip_from_model(camera: &Camera, model: &[f32; 16]) -> [f32; 16] {
    // Cancel the large city translation before projecting millimetre-scale
    // car detail. Adding it on the GPU first collapses nearby body panels.
    let mut relative = *model;
    for axis in 0..3 {
        relative[12 + axis] -= util::geometry::xyz_component(camera.eye, axis);
    }
    multiply(camera.relative_view_projection, relative)
}
#[cfg(test)]
fn project(camera: &Camera, point: XYZ) -> Option<[f32; 2]> {
    let m = camera.view_projection;
    let w = m[3] * point.x + m[7] * point.y + m[11] * point.z + m[15];
    if w <= 0.1 {
        return None;
    }
    Some(std::array::from_fn(|i| {
        (m[i] * point.x + m[i + 4] * point.y + m[i + 8] * point.z + m[i + 12]) / w
    }))
}

pub fn set_signal_time(program: u32, seconds: f32) {
    let name = CString::new("u_signal_time").unwrap();
    unsafe {
        gl::UseProgram(program);
        gl::Uniform1f(gl::GetUniformLocation(program, name.as_ptr()), seconds);
    }
}

pub fn chase_camera(aspect: f32, pose: render::camera::Pose) -> Camera {
    camera_at(aspect, pose.eye, pose.target, 0.0)
}

pub fn bonnet_camera(
    aspect: f32,
    pose: render::camera::Pose,
    car: &physics::car::Vehicle,
) -> Camera {
    let body = vehicle_body_matrix(car);
    camera_at_up(
        aspect,
        pose.eye,
        pose.target,
        0.0,
        XYZ {
            x: body[4],
            y: body[5],
            z: body[6],
        },
    )
}

pub fn garage_camera(
    aspect: f32,
    position: XYZ,
    angle: f32,
    city: Option<&model::world::WorldLocation>,
) -> Camera {
    let angle = angle + 0.8;
    let eye = XYZ {
        x: position.x + angle.sin() * 8.0,
        y: position.y + 3.6,
        z: position.z + angle.cos() * 8.0,
    };
    let eye = city.map_or(eye, |city| {
        render::camera::camera_eye(
            city,
            XYZ {
                x: position.x,
                y: position.y + 0.5,
                z: position.z,
            },
            eye,
        )
    });
    let mut camera = camera_at(
        aspect,
        eye,
        XYZ {
            x: position.x,
            y: position.y + 0.25,
            z: position.z,
        },
        0.0,
    );
    // Keep the preview above the two selection columns.
    for col in 0..4 {
        camera.view_projection[col * 4 + 1] += 0.52 * camera.view_projection[col * 4 + 3];
    }
    camera
}

fn camera_at(aspect: f32, eye: XYZ, target: XYZ, horizontal_shift: f32) -> Camera {
    camera_at_up(
        aspect,
        eye,
        target,
        horizontal_shift,
        XYZ {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        },
    )
}
fn camera_at_up(aspect: f32, eye: XYZ, target: XYZ, horizontal_shift: f32, up_axis: XYZ) -> Camera {
    let forward = normalize(XYZ {
        x: target.x - eye.x,
        y: target.y - eye.y,
        z: target.z - eye.z,
    });
    let right = normalize(cross(forward, up_axis));
    let up = cross(right, forward);
    let view = [
        right.x,
        up.x,
        -forward.x,
        0.0,
        right.y,
        up.y,
        -forward.y,
        0.0,
        right.z,
        up.z,
        -forward.z,
        0.0,
        -dot(right, eye),
        -dot(up, eye),
        dot(forward, eye),
        1.0,
    ];
    let mut projection = perspective(aspect, 1.0, 0.3, VIEW_DISTANCE);
    projection[8] = -horizontal_shift;
    let mut rotation = view;
    rotation[12..15].fill(0.0);
    Camera {
        view_projection: multiply(projection, view),
        relative_view_projection: multiply(projection, rotation),
        eye,
        particle_view: render::tire_particles::View {
            eye,
            right,
            up,
            forward,
        },
    }
}

/// Body transform shared by rendering and the bonnet camera mount.
pub fn vehicle_body_matrix(vehicle: &physics::car::Vehicle) -> [f32; 16] {
    let mut body_position = vehicle.position;
    body_position.y += vehicle.curb_bump;
    body_model_matrix(
        body_position,
        vehicle.heading,
        vehicle.suspension_pitch + vehicle.body_pitch,
        vehicle.suspension_roll + vehicle.body_roll,
    )
}

fn model_matrix(car_position: XYZ, vehicle_heading: f32, pitch: f32) -> [f32; 16] {
    body_model_matrix(car_position, vehicle_heading, pitch, 0.0)
}

fn body_model_matrix(car_position: XYZ, vehicle_heading: f32, pitch: f32, roll: f32) -> [f32; 16] {
    let c = vehicle_heading.cos();
    let s = vehicle_heading.sin();
    let (sp, cp) = pitch.sin_cos();
    let (sr, cr) = roll.sin_cos();
    let right = XYZ {
        x: c,
        y: 0.0,
        z: -s,
    };
    let up = XYZ {
        x: -s * sp,
        y: cp,
        z: -c * sp,
    };
    let forward = XYZ {
        x: s * cp,
        y: sp,
        z: c * cp,
    };
    let rolled_right: XYZ = util::geometry::xyz_from_fn(|i| {
        util::geometry::xyz_component(right, i) * cr + util::geometry::xyz_component(up, i) * sr
    });
    let rolled_up: XYZ = util::geometry::xyz_from_fn(|i| {
        util::geometry::xyz_component(up, i) * cr - util::geometry::xyz_component(right, i) * sr
    });
    [
        rolled_right.x,
        rolled_right.y,
        rolled_right.z,
        0.0,
        rolled_up.x,
        rolled_up.y,
        rolled_up.z,
        0.0,
        forward.x,
        forward.y,
        forward.z,
        0.0,
        car_position.x,
        car_position.y,
        car_position.z,
        1.0,
    ]
}

#[cfg(test)]
fn vehicle_wheel_matrix(car: &physics::car::Vehicle, body: [f32; 16], index: usize) -> [f32; 16] {
    multiply(body, vehicle_wheel_local_matrix(car, body, index))
}

fn vehicle_wheel_local_matrix(
    car: &physics::car::Vehicle,
    body: [f32; 16],
    index: usize,
) -> [f32; 16] {
    let mut center = car.spec.design.mounts[index];
    let mounted: XYZ = util::geometry::xyz_from_fn(|i| {
        body[i] * center.x + body[4 + i] * center.y + body[8 + i] * center.z + body[12 + i]
    });
    let ground = world::transport::drive_height_near(
        &car.track,
        util::geometry::xz(mounted),
        car.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
    );
    let radius = car.spec.design.wheel_radii[index];
    // Bounded compression/droop keeps tires following bumps while the body
    // heaves and tilts on its springs. Airborne wheels stop at full extension.
    center.y += ((ground + radius - mounted.y) / body[5].abs().max(0.5)).clamp(-0.16, 0.14);
    let front = index >= 2;
    wheel_matrix(
        model_matrix(
            XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
            0.0,
        ),
        center,
        if front {
            physics::car::front_wheel_angle(car)
        } else {
            0.0
        },
        car.wheel_phase[usize::from(front)],
    )
}

fn wheel_matrix(body: [f32; 16], center: XYZ, steer: f32, spin: f32) -> [f32; 16] {
    let mount = model_matrix(center, steer, 0.0);
    let roll = model_matrix(
        XYZ {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        0.0,
        -spin,
    );
    multiply(multiply(body, mount), roll)
}

fn dot(a: XYZ, b: XYZ) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}
fn cross(a: XYZ, b: XYZ) -> XYZ {
    XYZ {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    }
}
fn normalize(v: XYZ) -> XYZ {
    let length = dot(v, v).sqrt();
    XYZ {
        x: v.x / length,
        y: v.y / length,
        z: v.z / length,
    }
}
fn multiply(a: [f32; 16], b: [f32; 16]) -> [f32; 16] {
    let mut out = [0.0; 16];
    for column in 0..4 {
        for row in 0..4 {
            out[column * 4 + row] = (0..4).map(|i| a[i * 4 + row] * b[column * 4 + i]).sum();
        }
    }
    out
}

pub fn load_gl() {
    unsafe {
        extern "C" {
            fn SDL_GL_GetProcAddress(name: *const i8) -> *mut std::ffi::c_void;
        }
        gl::load_with(|s: &str| {
            let c_str = CString::new(s).unwrap();
            SDL_GL_GetProcAddress(c_str.as_ptr()) as *const std::ffi::c_void
        });
    }
}

pub fn create_gl_context(window: &Window) -> Result<sdl3::video::GLContext, String> {
    let context = window.gl_create_context().map_err(|e| e.to_string())?;
    window
        .gl_make_current(&context)
        .map_err(|e| e.to_string())?;
    Ok(context)
}

pub fn swap_window(window: &Window) {
    window.gl_swap_window();
}

pub fn debug_context_info() -> String {
    const GL_MAJOR_VERSION: u32 = 0x821B;
    const GL_MINOR_VERSION: u32 = 0x821C;
    const GL_CONTEXT_PROFILE_MASK: u32 = 0x9126;
    const GL_CONTEXT_FLAGS: u32 = 0x821E;
    unsafe {
        gl::ClearColor(0.0, 0.0, 0.0, 1.0);
        gl::Clear(0x0000_4000);
        let mut major: i32 = -1;
        let mut minor: i32 = -1;
        let mut ctx_profile_mask: i32 = -1;
        let mut ctx_flags: i32 = -1;
        gl::GetIntegerv(GL_MAJOR_VERSION, &mut major);
        gl::GetIntegerv(GL_MINOR_VERSION, &mut minor);
        gl::GetIntegerv(GL_CONTEXT_FLAGS, &mut ctx_flags);
        gl::GetIntegerv(GL_CONTEXT_PROFILE_MASK, &mut ctx_profile_mask);
        let error: u32 = gl::GetError();
        format!(
            "GL version {}.{} profile_mask={:#x} flags={:#x} error={:#x}",
            major, minor, ctx_profile_mask, ctx_flags, error
        )
    }
}

pub fn set_viewport(w: i32, h: i32) {
    unsafe {
        gl::Viewport(0, 0, w, h);
    }
}

pub fn clear_screen() {
    unsafe {
        gl::ClearColor(0.57, 0.73, 0.80, 1.0);
        gl::Clear(0x0000_4000 | 0x0000_0100);
    }
}

pub fn enable_depth_test() {
    unsafe {
        gl::Enable(GL_DEPTH_TEST);
        gl::Enable(gl::MULTISAMPLE);
    }
}

#[cfg(test)]
#[path = "tests/gl.rs"]
mod tests;

#[cfg(test)]
#[path = "tests/gl_renderer.rs"]
mod renderer_tests;
