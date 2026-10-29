//! OpenGL scene/HUD programs, mesh upload, and chase-camera math.

use std::ffi::CString;

use sdl3::video::Window;

pub const VS_SRC: &str = "#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;
uniform mat4 u_mvp;
out vec3 v_color;
void main() { gl_Position = u_mvp * vec4(position, 1.0); v_color = color; }";

pub const FS_SRC: &str = "#version 330 core
out vec4 o;
in vec3 v_color;
uniform vec3 u_tint;
void main() { o = vec4(v_color * u_tint, 1.0); }";

pub const SCENE_VS: &str = "#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;
layout (location = 2) in vec3 normal;
layout (location = 3) in vec2 material;
uniform mat4 u_view_projection;
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
    gl_Position = u_view_projection * world;
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
uniform int u_city_reference;
uniform sampler2DArray u_aerial;
uniform int u_aerial_count;
uniform vec4 u_aerial_bounds[48];
uniform int u_aerial_layers[48];
uniform vec2 u_aerial_uv;
out vec4 o;
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
    // Reserved material -2: emissive signal lenses. The alpha-free material
    // carries an integer program offset and lens index, with a fractional guard.
    if (v_material.x < -1.5) {
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
                float road=min(min(distance(v_color,vec3(0.18,0.20,0.22)),distance(v_color,vec3(0.25,0.26,0.26))),min(distance(v_color,vec3(0.32,0.31,0.28)),distance(v_color,vec3(0.42,0.43,0.40))));
                if (road<0.006) colour*=0.65;
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
    vec3 paint = v_color * mix(vec3(1.0), u_tint, clamp(v_material.x, 0.0, 1.0));
    vec3 ambient = mix(vec3(0.25, 0.27, 0.23), vec3(0.53, 0.61, 0.67), n.y * 0.5 + 0.5);
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

pub struct Shader {
    pub handle: gl::types::GLuint,
}

impl Shader {
    pub fn compile(src: &str, kind: u32) -> Result<Shader, String> {
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
}

impl Drop for Shader {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteShader(self.handle);
        }
    }
}

pub fn create_program(vs_src: &str, fs_src: &str) -> Result<u32, String> {
    let vs = Shader::compile(vs_src, GL_VERTEX_SHADER)?;
    let fs = Shader::compile(fs_src, GL_FRAGMENT_SHADER)?;
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
impl CityBuffer {
    fn upload(vertices: &[crate::scene::Vertex]) -> Self {
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
            Self {
                vao,
                vbo,
                count: vertices.len() as i32,
            }
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
    entry: crate::scene_archive::Entry,
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
    archive: std::cell::RefCell<Option<crate::scene_archive::Archive>>,
    prefetch: Option<crate::prefetch::Prefetch>,
    aerial: std::cell::RefCell<Option<crate::aerial::Aerial>>,
    visible_chunks: std::cell::Cell<usize>,
    car_ranges: [std::ops::Range<i32>; 3],
    wheel_range: std::ops::Range<i32>,
}

impl Drop for Mesh {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteBuffers(1, &self.vbo);
            gl::DeleteVertexArrays(1, &self.vao);
        }
    }
}

impl Mesh {
    pub fn new(track: &crate::track::Track) -> Result<Self, String> {
        let archive = if track.city().is_some() {
            Some(crate::scene_archive::Archive::open_default().map_err(|e| e.to_string())?)
        } else {
            None
        };
        let prefetch = archive
            .as_ref()
            .map(|a| a.fork().map(crate::prefetch::Prefetch::new))
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
            let crate::scene::Geometry {
                vertices: verts,
                world_count: ground_vertex_count,
                car_ranges,
                wheel_range,
            } = crate::scene::build(track);
            gl::BufferData(
                GL_ARRAY_BUFFER,
                (verts.len() * std::mem::size_of::<crate::scene::Vertex>()) as isize,
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
        Ok(Mesh {
            vao,
            vbo,
            ground_vertex_count,
            city_chunks: std::cell::RefCell::new(city_chunks),
            archive: std::cell::RefCell::new(archive),
            prefetch,
            aerial: std::cell::RefCell::new(if track.city().is_some() {
                crate::aerial::Aerial::open()
            } else {
                None
            }),
            visible_chunks: std::cell::Cell::new(0),
            car_ranges,
            wheel_range,
        })
    }

    pub fn draw(
        &self,
        program: u32,
        camera: &Camera,
        model: &[f32; 16],
        tint: [f32; 3],
        car: &crate::vehicle::Vehicle,
    ) {
        unsafe {
            gl::BindVertexArray(self.vao);
            gl::UseProgram(program);
            set_scene_transform(program, camera, model);
            set_tint(program, tint);
            let range = &self.car_ranges[car.model.index()];
            gl::DrawArrays(GL_TRIANGLES, range.start, range.end - range.start);
            set_tint(program, [1.0; 3]);
            for (index, center) in car.model.wheel_centers().into_iter().enumerate() {
                let front = index >= 2;
                let steer = if front { car.front_wheel_angle() } else { 0.0 };
                let matrix =
                    wheel_matrix(*model, center, steer, car.wheel_phase[usize::from(front)]);
                set_scene_transform(program, camera, &matrix);
                gl::DrawArrays(
                    GL_TRIANGLES,
                    self.wheel_range.start,
                    self.wheel_range.end - self.wheel_range.start,
                );
            }
        }
    }

    fn plan_prefetch(&self, camera: &Camera) {
        let Some(prefetch) = &self.prefetch else {
            return;
        };
        let chunks = self.city_chunks.borrow();
        let mut wanted = Vec::new();
        for (index, chunk) in chunks.iter().enumerate() {
            let distance = crate::chunks::distance_squared(chunk.entry.bounds, camera.eye);
            if distance > (VIEW_DISTANCE + 128.0).powi(2) {
                continue;
            }
            let visible = crate::chunks::visible(chunk.entry.bounds, camera.view_projection);
            let coarse = coarse_lod(chunk.gpu.as_ref().map(|g| g.coarse), distance);
            let rank = if visible { 0 } else { 2 };
            if chunk.gpu.is_none() {
                wanted.push((rank, distance, (index, 0)));
                wanted.push((rank, distance, (index, if coarse { 2 } else { 1 })));
            } else if chunk.gpu.as_ref().unwrap().coarse != coarse {
                wanted.push((0, distance, (index, if coarse { 2 } else { 1 })));
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
        prefetch.plan(wanted.into_iter().map(|v| v.2).collect());
    }
    fn city_layer(
        &self,
        archive: &mut crate::scene_archive::Archive,
        index: usize,
        layer: usize,
    ) -> Vec<crate::scene::Vertex> {
        if let Some(result) = self.prefetch.as_ref().and_then(|p| p.take((index, layer))) {
            return result.expect("prefetch city geometry");
        }
        archive
            .read_layer(index, layer)
            .expect("read city geometry")
    }
    pub fn draw_ground(
        &self,
        program: u32,
        camera: &Camera,
        contacts: &[crate::shadows::ContactShadow],
    ) {
        self.plan_prefetch(camera);
        unsafe {
            gl::BindVertexArray(self.vao);
            gl::UseProgram(program);
            let aerial_count = CString::new("u_aerial_count").unwrap();
            gl::Uniform1i(gl::GetUniformLocation(program, aerial_count.as_ptr()), 0);
            if let Some(aerial) = self.aerial.borrow_mut().as_mut() {
                aerial.bind(program, camera.eye);
            }
            let name = CString::new("u_city_reference").unwrap();
            gl::Uniform1i(
                gl::GetUniformLocation(program, name.as_ptr()),
                self.prefetch.is_some() as i32,
            );
            set_scene_transform(program, camera, &model_matrix([0.0; 3], 0.0, 0.0));
            set_tint(program, [1.0; 3]);
            set_contacts(program, contacts);
            gl::DrawArrays(GL_TRIANGLES, 0, self.ground_vertex_count);
            let mut visible = 0;
            let mut archive = self.archive.borrow_mut();
            for (index, chunk) in self.city_chunks.borrow_mut().iter_mut().enumerate() {
                if crate::chunks::distance_squared(chunk.entry.bounds, camera.eye)
                    > (VIEW_DISTANCE + crate::chunks::SIZE).powi(2)
                {
                    chunk.gpu = None;
                    continue;
                }
                if !crate::chunks::visible(chunk.entry.bounds, camera.view_projection) {
                    continue;
                }
                let coarse = coarse_lod(
                    chunk.gpu.as_ref().map(|g| g.coarse),
                    crate::chunks::distance_squared(chunk.entry.bounds, camera.eye),
                );
                let archive = archive.as_mut().expect("city archive");
                if chunk.gpu.is_none() {
                    let ground = {
                        let vertices = self.city_layer(archive, index, 0);
                        CityBuffer::upload(&vertices)
                    };
                    let buildings = {
                        let vertices = self.city_layer(archive, index, if coarse { 2 } else { 1 });
                        CityBuffer::upload(&vertices)
                    };
                    chunk.gpu = Some(CityGpu {
                        ground,
                        buildings,
                        coarse,
                    });
                }
                let gpu = chunk.gpu.as_mut().unwrap();
                if gpu.coarse != coarse {
                    let vertices = self.city_layer(archive, index, if coarse { 2 } else { 1 });
                    gpu.buildings = CityBuffer::upload(&vertices);
                    gpu.coarse = coarse;
                }
                for buffer in [&gpu.ground, &gpu.buildings] {
                    gl::BindVertexArray(buffer.vao);
                    gl::DrawArrays(GL_TRIANGLES, 0, buffer.count);
                }
                visible += 1;
            }
            self.visible_chunks.set(visible);
        }
    }
    #[cfg(test)]
    pub fn check_chunk_residency(&self, camera: &Camera) {
        for chunk in self.city_chunks.borrow().iter() {
            if crate::chunks::distance_squared(chunk.entry.bounds, camera.eye)
                > (VIEW_DISTANCE + crate::chunks::SIZE).powi(2)
            {
                assert!(chunk.gpu.is_none(), "distant GPU buffer was not released");
            } else if crate::chunks::visible(chunk.entry.bounds, camera.view_projection) {
                assert!(chunk.gpu.is_some(), "visible chunk was not uploaded");
            }
        }
    }
    #[cfg(test)]
    pub fn prefetched_ground_location(&self) -> Option<[f32; 2]> {
        let key = self.prefetch.as_ref()?.wait_for_ground()?;
        let chunks = self.city_chunks.borrow();
        let bounds = chunks[key.0].entry.bounds;
        Some([
            (bounds[0][0] + bounds[1][0]) * 0.5,
            (bounds[0][2] + bounds[1][2]) * 0.5,
        ])
    }
    #[cfg(test)]
    pub fn prefetch_stats(&self) -> (usize, usize, usize, usize, usize) {
        self.prefetch
            .as_ref()
            .map_or((0, 0, 0, 0, 0), |p| p.stats())
    }
    #[cfg(test)]
    pub fn paging_stats(&self) -> (usize, usize, usize) {
        let archive = self.archive.borrow();
        let Some(archive) = archive.as_ref() else {
            return (0, 0, 0);
        };
        (
            archive.bytes_read,
            archive.peak_decode_bytes,
            self.city_chunks
                .borrow()
                .iter()
                .filter(|c| c.gpu.as_ref().is_some_and(|g| g.coarse))
                .count(),
        )
    }
    #[cfg(test)]
    pub fn chunk_stats(&self) -> (usize, usize, usize, usize) {
        let chunks = self.city_chunks.borrow();
        (
            chunks.len(),
            chunks.iter().filter(|c| c.gpu.is_some()).count(),
            self.visible_chunks.get(),
            chunks
                .iter()
                .filter(|c| c.gpu.is_some())
                .map(|c| {
                    let gpu = c.gpu.as_ref().unwrap();
                    (gpu.ground.count + gpu.buildings.count) as usize
                        * std::mem::size_of::<crate::scene::Vertex>()
                })
                .sum(),
        )
    }
}

unsafe fn set_contacts(program: u32, contacts: &[crate::shadows::ContactShadow]) {
    use crate::shadows::{MAX_CONTACTS, STRENGTH};
    let count = contacts.len().min(MAX_CONTACTS);
    let mut origins = [[0.0; 3]; MAX_CONTACTS];
    let mut axes = [[0.0; 4]; MAX_CONTACTS];
    for (i, contact) in contacts.iter().take(count).enumerate() {
        origins[i] = contact.origin;
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
    let stride = std::mem::size_of::<crate::scene::Vertex>() as i32;
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

impl SkidRenderer {
    pub fn new() -> Self {
        unsafe {
            let (mut vao, mut vbo) = (0, 0);
            gl::GenVertexArrays(1, &mut vao);
            gl::GenBuffers(1, &mut vbo);
            gl::BindVertexArray(vao);
            gl::BindBuffer(GL_ARRAY_BUFFER, vbo);
            scene_attributes();
            gl::BindVertexArray(0);
            Self { vao, vbo }
        }
    }

    pub fn draw(&self, program: u32, camera: &Camera, marks: &crate::feedback::SkidMarks) {
        let vertices = marks.vertices();
        if vertices.is_empty() {
            return;
        }
        unsafe {
            gl::UseProgram(program);
            gl::BindVertexArray(self.vao);
            gl::BindBuffer(GL_ARRAY_BUFFER, self.vbo);
            gl::BufferData(
                GL_ARRAY_BUFFER,
                std::mem::size_of_val(vertices.as_slice()) as isize,
                vertices.as_ptr() as *const _,
                gl::STREAM_DRAW,
            );
            set_scene_transform(program, camera, &model_matrix([0.0; 3], 0.0, 0.0));
            set_tint(program, [1.0; 3]);
            gl::DepthMask(gl::FALSE);
            gl::DrawArrays(GL_TRIANGLES, 0, vertices.len() as i32);
            gl::DepthMask(gl::TRUE);
        }
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

impl ParticleRenderer {
    pub fn new() -> Result<Self, String> {
        let program = create_program(PARTICLE_VS, PARTICLE_FS)?;
        unsafe {
            let (mut vao, mut vbo) = (0, 0);
            gl::GenVertexArrays(1, &mut vao);
            gl::GenBuffers(1, &mut vbo);
            gl::BindVertexArray(vao);
            gl::BindBuffer(GL_ARRAY_BUFFER, vbo);
            let stride = std::mem::size_of::<crate::particles::Vertex>() as i32;
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
            Ok(Self { vao, vbo, program })
        }
    }

    pub fn draw(&self, camera: &Camera, particles: &crate::particles::TireParticles) -> usize {
        let vertices = particles.vertices(camera.particle_view);
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
            gl::UseProgram(self.program);
            gl::BindVertexArray(self.vao);
            gl::BindBuffer(GL_ARRAY_BUFFER, self.vbo);
            gl::BufferData(
                GL_ARRAY_BUFFER,
                std::mem::size_of_val(vertices.as_slice()) as isize,
                vertices.as_ptr() as *const _,
                gl::STREAM_DRAW,
            );
            let name = CString::new("u_view_projection").unwrap();
            gl::UniformMatrix4fv(
                gl::GetUniformLocation(self.program, name.as_ptr()),
                1,
                GL_FALSE,
                camera.view_projection.as_ptr(),
            );
            let name = CString::new("u_eye").unwrap();
            gl::Uniform3f(
                gl::GetUniformLocation(self.program, name.as_ptr()),
                camera.eye[0],
                camera.eye[1],
                camera.eye[2],
            );
            set_fog(self.program);
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
    for (name, matrix) in [
        ("u_view_projection", &camera.view_projection),
        ("u_model", model),
    ] {
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
        camera.eye[0],
        camera.eye[1],
        camera.eye[2],
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

pub fn perspective(aspect: f32, fov_y: f32, near: f32, far: f32) -> [f32; 16] {
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
    eye: [f32; 3],
    particle_view: crate::particles::View,
}

pub fn set_signal_time(program: u32, seconds: f32) {
    let name = CString::new("u_signal_time").unwrap();
    unsafe {
        gl::UseProgram(program);
        gl::Uniform1f(gl::GetUniformLocation(program, name.as_ptr()), seconds);
    }
}

#[cfg(test)]
pub fn test_pixel(camera: &Camera, point: [f32; 3], width: i32, height: i32) -> [i32; 2] {
    let m = camera.view_projection;
    let clip: [f32; 4] = std::array::from_fn(|i| {
        m[i] * point[0] + m[4 + i] * point[1] + m[8 + i] * point[2] + m[12 + i]
    });
    [
        ((clip[0] / clip[3] * 0.5 + 0.5) * width as f32) as i32,
        ((clip[1] / clip[3] * 0.5 + 0.5) * height as f32) as i32,
    ]
}

pub fn chase_camera(aspect: f32, pose: crate::camera::Pose) -> Camera {
    camera_at(aspect, pose.eye, pose.target, 0.0)
}

pub fn garage_camera(
    aspect: f32,
    position: [f32; 3],
    angle: f32,
    city: Option<&crate::city::City>,
) -> Camera {
    let angle = angle + 0.8;
    let eye = [
        position[0] + angle.sin() * 8.0,
        position[1] + 3.6,
        position[2] + angle.cos() * 8.0,
    ];
    let eye = city.map_or(eye, |city| {
        city.camera_eye([position[0], position[1] + 0.5, position[2]], eye)
    });
    camera_at(
        aspect,
        eye,
        [position[0], position[1] + 0.25, position[2]],
        0.43,
    )
}

fn camera_at(aspect: f32, eye: [f32; 3], target: [f32; 3], horizontal_shift: f32) -> Camera {
    let forward = normalize([target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]]);
    let right = normalize(cross(forward, [0.0, 1.0, 0.0]));
    let up = cross(right, forward);
    let view = [
        right[0],
        up[0],
        -forward[0],
        0.0,
        right[1],
        up[1],
        -forward[1],
        0.0,
        right[2],
        up[2],
        -forward[2],
        0.0,
        -dot(right, eye),
        -dot(up, eye),
        dot(forward, eye),
        1.0,
    ];
    let mut projection = perspective(aspect, 1.0, 0.3, VIEW_DISTANCE);
    projection[8] = -horizontal_shift;
    Camera {
        view_projection: multiply(projection, view),
        eye,
        particle_view: crate::particles::View {
            eye,
            right,
            up,
            forward,
        },
    }
}

pub fn model_matrix(car_position: [f32; 3], vehicle_heading: f32, pitch: f32) -> [f32; 16] {
    body_model_matrix(car_position, vehicle_heading, pitch, 0.0)
}

pub fn body_model_matrix(
    car_position: [f32; 3],
    vehicle_heading: f32,
    pitch: f32,
    roll: f32,
) -> [f32; 16] {
    let c = vehicle_heading.cos();
    let s = vehicle_heading.sin();
    let (sp, cp) = pitch.sin_cos();
    let (sr, cr) = roll.sin_cos();
    let right = [c, 0.0, -s];
    let up = [-s * sp, cp, -c * sp];
    let forward = [s * cp, sp, c * cp];
    let rolled_right: [f32; 3] = std::array::from_fn(|i| right[i] * cr + up[i] * sr);
    let rolled_up: [f32; 3] = std::array::from_fn(|i| up[i] * cr - right[i] * sr);
    [
        rolled_right[0],
        rolled_right[1],
        rolled_right[2],
        0.0,
        rolled_up[0],
        rolled_up[1],
        rolled_up[2],
        0.0,
        forward[0],
        forward[1],
        forward[2],
        0.0,
        car_position[0],
        car_position[1],
        car_position[2],
        1.0,
    ]
}

fn wheel_matrix(body: [f32; 16], center: [f32; 3], steer: f32, spin: f32) -> [f32; 16] {
    let mount = model_matrix(center, steer, 0.0);
    let roll = model_matrix([0.0; 3], 0.0, -spin);
    multiply(multiply(body, mount), roll)
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn normalize(v: [f32; 3]) -> [f32; 3] {
    let length = dot(v, v).sqrt();
    [v[0] / length, v[1] / length, v[2] / length]
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
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn building_lod_uses_hysteresis_to_avoid_boundary_flicker() {
        assert!(!super::coarse_lod(Some(false), 195.0_f32.powi(2)));
        assert!(super::coarse_lod(Some(false), 205.0_f32.powi(2)));
        assert!(super::coarse_lod(Some(true), 170.0_f32.powi(2)));
        assert!(!super::coarse_lod(Some(true), 155.0_f32.powi(2)));
    }

    use super::*;

    fn transform(m: [f32; 16], p: [f32; 3]) -> [f32; 3] {
        std::array::from_fn(|i| m[i] * p[0] + m[4 + i] * p[1] + m[8 + i] * p[2] + m[12 + i])
    }

    #[test]
    fn closer_default_camera_keeps_the_car_in_frame_at_speed_and_in_reverse() {
        for id in crate::track::ALL {
            for model in crate::cars::ALL {
                let track = id.track();
                let road = track.spawn();
                let mut car = crate::vehicle::Vehicle::on_track(
                    road.lane_position(0.0),
                    road.heading(),
                    model,
                    track,
                );
                for speed in [-6.0, 0.0, 30.0, 45.0] {
                    car.speed = speed;
                    car.velocity = road.tangent.map(|n| n * speed);
                    let chase = crate::camera::ChaseCamera::new(&car);
                    let camera = chase_camera(1280.0 / 720.0, chase.pose());
                    let matrix = multiply(
                        camera.view_projection,
                        model_matrix(car.position, car.heading, 0.0),
                    );
                    for x in [-1.4, 1.4] {
                        for y in [-0.42, 1.25] {
                            for z in [-2.2, 2.2] {
                                let p = [x, y, z];
                                let clip = transform(matrix, p);
                                let w = matrix[3] * x + matrix[7] * y + matrix[11] * z + matrix[15];
                                assert!(w > 0.0);
                                assert!(
                                    clip[0].abs() / w < 0.98 && clip[1].abs() / w < 0.98,
                                    "car clipped for {id:?}/{model:?} at {speed}: {:?}",
                                    clip.map(|v| v / w)
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn wheels_turn_about_their_axles_not_the_vehicle_origin() {
        for car in crate::cars::ALL {
            for center in car.wheel_centers() {
                let body = model_matrix([12.0, 3.0, -7.0], 1.2, 0.15);
                let axle = transform(body, center);
                for (steer, spin) in [(0.0, 0.0), (-0.4, 1.5), (0.4, 5.0)] {
                    let matrix = wheel_matrix(body, center, steer, spin);
                    let origin = transform(matrix, [0.0; 3]);
                    for i in 0..3 {
                        assert!((origin[i] - axle[i]).abs() < 0.0001);
                    }
                    let rim = transform(matrix, [0.0, crate::cars::WHEEL_RADIUS, 0.0]);
                    let radius = (0..3)
                        .map(|i| (rim[i] - axle[i]).powi(2))
                        .sum::<f32>()
                        .sqrt();
                    assert!((radius - crate::cars::WHEEL_RADIUS).abs() < 0.0001);
                }
            }
        }
    }

    #[test]
    fn body_matrix_adds_roll_without_moving_the_car_or_scaling_axes() {
        let origin = [12.0, 3.0, -7.0];
        let flat = model_matrix(origin, 0.4, 0.0);
        let rolled = body_model_matrix(origin, 0.4, 0.0, 0.08);
        assert_eq!(transform(rolled, [0.0; 3]), origin);
        assert_ne!(flat[1], rolled[1], "roll should tilt the local right axis");
        for axis in 0..3 {
            let length = (0..3)
                .map(|row| rolled[axis * 4 + row].powi(2))
                .sum::<f32>()
                .sqrt();
            assert!((length - 1.0).abs() < 0.0001);
        }
        let right_dot_up: f32 = (0..3).map(|row| rolled[row] * rolled[4 + row]).sum();
        assert!(right_dot_up.abs() < 0.0001);
    }
}

#[cfg(test)]
#[path = "aerial_preview.rs"]
mod aerial_preview;
