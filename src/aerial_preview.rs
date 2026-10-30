//! Aerial-ground experiment with the normal 3D city, isolated to the preview test.
use super::*;
use std::path::Path;

const VS: &str = "#version 330 core
layout(location=0) in vec3 position;
layout(location=1) in vec2 uv;
uniform mat4 u_view_projection;
out vec2 v_uv;
void main(){v_uv=uv;gl_Position=u_view_projection*vec4(position,1.0);}";
const FS: &str = "#version 330 core
in vec2 v_uv;
uniform sampler2D aerial;
out vec4 o;
void main(){o=vec4(texture(aerial,v_uv).rgb,1.0);}";

fn local(mx: f64, my: f64) -> [f32; 2] {
    let lat = (2.0 * (my / 6_378_137.0).exp().atan() - std::f64::consts::FRAC_PI_2).to_degrees();
    let lon = (mx / 6_378_137.0).to_degrees();
    [
        ((lon - 33.320).to_radians() * 6_371_000.0 * 35.162_f64.to_radians().cos()) as f32,
        (-(lat - 35.162).to_radians() * 6_371_000.0) as f32,
    ]
}
fn mercator(p: [f32; 2]) -> [f64; 2] {
    let lat = 35.162_f64.to_radians() - p[1] as f64 / 6_371_000.0;
    let lon = 33.320_f64.to_radians() + p[0] as f64 / (6_371_000.0 * 35.162_f64.to_radians().cos());
    [
        6_378_137.0 * lon,
        6_378_137.0 * (std::f64::consts::FRAC_PI_4 + lat * 0.5).tan().ln(),
    ]
}

#[test]
#[ignore = "requires SDL/OpenGL and the explicit local aerial preview image"]
fn aerial_drape_preview() {
    let directory = std::env::var("NFS_AERIAL_PREVIEW_DIR").expect("set NFS_AERIAL_PREVIEW_DIR");
    let directory = Path::new(&directory);
    let values: Vec<f64> = std::fs::read_to_string(directory.join("georef.txt"))
        .unwrap()
        .split_whitespace()
        .map(|s| s.parse().unwrap())
        .collect();
    assert_eq!(values.len(), 6);
    let [width, height, xmin, ymin, xmax, ymax]: [f64; 6] = values.try_into().unwrap();
    let pixels = std::fs::read(directory.join("image.rgb")).unwrap();
    assert_eq!(pixels.len(), width as usize * height as usize * 3);
    let mut app = crate::App::new();
    app.key_down(sdl3::keyboard::Keycode::F7);
    let city_track = app.session.vehicle.track.clone();
    let city = city_track.city().unwrap();
    let field = city.elevation.as_ref().unwrap();
    let nw = local(xmin, ymax);
    let se = local(xmax, ymin);
    let hybrid = std::env::var_os("NFS_AERIAL_RAW").is_none();
    let mut vertices = Vec::<f32>::new();
    let mut emit = |tri: [[f32; 3]; 3]| {
        for p in tri {
            let m = mercator([p[0], p[2]]);
            vertices.extend([
                p[0],
                p[1],
                p[2],
                ((m[0] - xmin) / (xmax - xmin)) as f32,
                ((ymax - m[1]) / (ymax - ymin)) as f32,
            ]);
        }
    };
    field.drape([nw, [se[0], nw[1]], se], &mut emit);
    field.drape([nw, se, [nw[0], se[1]]], &mut emit);
    let program = create_program(VS, FS).unwrap();
    let (mut vao, mut vbo, mut texture) = (0, 0, 0);
    unsafe {
        ::gl::GenVertexArrays(1, &mut vao);
        ::gl::BindVertexArray(vao);
        ::gl::GenBuffers(1, &mut vbo);
        ::gl::BindBuffer(::gl::ARRAY_BUFFER, vbo);
        ::gl::BufferData(
            ::gl::ARRAY_BUFFER,
            (vertices.len() * 4) as isize,
            vertices.as_ptr().cast(),
            ::gl::STATIC_DRAW,
        );
        for (slot, size, offset) in [(0, 3, 0), (1, 2, 12)] {
            ::gl::EnableVertexAttribArray(slot);
            ::gl::VertexAttribPointer(
                slot,
                size,
                ::gl::FLOAT,
                ::gl::FALSE,
                20,
                offset as *const std::ffi::c_void,
            );
        }
        ::gl::GenTextures(1, &mut texture);
        ::gl::ActiveTexture(::gl::TEXTURE0);
        ::gl::BindTexture(::gl::TEXTURE_2D, texture);
        ::gl::PixelStorei(::gl::UNPACK_ALIGNMENT, 1);
        ::gl::TexImage2D(
            ::gl::TEXTURE_2D,
            0,
            ::gl::RGB8 as i32,
            width as i32,
            height as i32,
            0,
            ::gl::RGB,
            ::gl::UNSIGNED_BYTE,
            pixels.as_ptr().cast(),
        );
        ::gl::GenerateMipmap(::gl::TEXTURE_2D);
        ::gl::TexParameteri(
            ::gl::TEXTURE_2D,
            ::gl::TEXTURE_MIN_FILTER,
            ::gl::LINEAR_MIPMAP_LINEAR as i32,
        );
        ::gl::TexParameteri(
            ::gl::TEXTURE_2D,
            ::gl::TEXTURE_MAG_FILTER,
            ::gl::LINEAR as i32,
        );
        // Ground textures are viewed at grazing angles. Preserve detail along
        // the short pixel footprint instead of selecting an overly blurry mip.
        let mut extensions = 0;
        ::gl::GetIntegerv(::gl::NUM_EXTENSIONS, &mut extensions);
        let anisotropic = (0..extensions).any(|i| {
            let name = ::gl::GetStringi(::gl::EXTENSIONS, i as u32);
            !name.is_null()
                && matches!(
                    std::ffi::CStr::from_ptr(name.cast()).to_bytes(),
                    b"GL_EXT_texture_filter_anisotropic" | b"GL_ARB_texture_filter_anisotropic"
                )
        });
        if anisotropic {
            let mut maximum = 1.0;
            ::gl::GetFloatv(0x84FF, &mut maximum);
            ::gl::TexParameterf(::gl::TEXTURE_2D, 0x84FE, maximum.min(16.0));
        }
        for axis in [::gl::TEXTURE_WRAP_S, ::gl::TEXTURE_WRAP_T] {
            ::gl::TexParameteri(::gl::TEXTURE_2D, axis, ::gl::CLAMP_TO_EDGE as i32);
        }
    }
    let ground = city.height([0.0, 0.0]);
    let road = city.recovery([-180.0, 0.0, 70.0]);
    let p = [
        road.point[0],
        if hybrid {
            city.drive_height([road.point[0], road.point[2]]) + 0.48
        } else {
            city.height([road.point[0], road.point[2]]) + 0.43
        },
        road.point[2],
    ];
    app.session.vehicle.reset(p, road.heading());
    let heading = road.heading();
    let (sin, cos) = heading.sin_cos();
    let curb = city
        .surfaces
        .as_ref()
        .unwrap()
        .curbs
        .iter()
        .filter(|c| (c.ends[1][0] - c.ends[0][0]).hypot(c.ends[1][1] - c.ends[0][1]) > 4.0)
        .min_by(|a, b| {
            let distance = |c: &crate::street_surface::Curb| {
                ((c.ends[0][0] + c.ends[1][0]) * 0.5 - p[0])
                    .hypot((c.ends[0][1] + c.ends[1][1]) * 0.5 - p[2])
            };
            distance(a).total_cmp(&distance(b))
        })
        .unwrap();
    let q = [
        (curb.ends[0][0] + curb.ends[1][0]) * 0.5,
        (curb.ends[0][1] + curb.ends[1][1]) * 0.5,
    ];
    let y = city.height(q);
    let normal = curb.inward;
    let views = [
        (
            "overview",
            crate::camera::Pose {
                eye: [175.0, ground + 245.0, 210.0],
                target: [0.0, ground, 0.0],
            },
            false,
        ),
        (
            "chase",
            crate::camera::Pose {
                eye: [p[0] - sin * 7.0, p[1] + 3.4, p[2] - cos * 7.0],
                target: [p[0] + sin * 12.0, p[1] + 0.4, p[2] + cos * 12.0],
            },
            true,
        ),
        (
            "oblique",
            crate::camera::Pose {
                eye: [p[0] - sin * 28.0, p[1] + 24.0, p[2] - cos * 28.0],
                target: [p[0] + sin * 18.0, p[1], p[2] + cos * 18.0],
            },
            true,
        ),
        (
            "curb",
            crate::camera::Pose {
                eye: [
                    q[0] + normal[0] * 3.0 - normal[1] * 3.0,
                    y + 0.65,
                    q[1] + normal[1] * 3.0 + normal[0] * 3.0,
                ],
                target: [q[0], y + 0.10, q[1]],
            },
            false,
        ),
        (
            "street",
            crate::camera::Pose {
                eye: [p[0], p[1] + 1.25, p[2]],
                target: [p[0] + sin * 35.0, p[1] + 1.25, p[2] + cos * 35.0],
            },
            false,
        ),
    ];
    for (name, pose, car) in views {
        let camera = chase_camera(
            crate::WINDOW_WIDTH as f32 / crate::WINDOW_HEIGHT as f32,
            pose,
        );
        clear_screen();
        if hybrid {
            app.mesh().draw_ground(app.scene_program, &camera, &[]);
            assert!(app.mesh().chunk_stats().2 > 0, "3D city must be visible");
            assert!(
                app.mesh().aerial.borrow().as_ref().unwrap().stats().0 > 0,
                "runtime imagery must be resident"
            );
        }
        if !hybrid {
            unsafe {
                ::gl::UseProgram(program);
                ::gl::BindVertexArray(vao);
                ::gl::UniformMatrix4fv(
                    ::gl::GetUniformLocation(program, c"u_view_projection".as_ptr()),
                    1,
                    ::gl::FALSE,
                    camera.view_projection.as_ptr(),
                );
                ::gl::Uniform1i(::gl::GetUniformLocation(program, c"aerial".as_ptr()), 0);
                // Culling is unnecessary for this two-sided diagnostic terrain patch.
                ::gl::Disable(::gl::CULL_FACE);
                ::gl::DrawArrays(::gl::TRIANGLES, 0, (vertices.len() / 5) as i32);
            }
        }
        if car {
            let h = |dx: f32, dz: f32| city.height([p[0] + dx, p[2] + dz]);
            let pitch = ((h(sin * 1.3, cos * 1.3) - h(-sin * 1.3, -cos * 1.3)) / 2.6).atan();
            let roll = ((h(cos * 0.8, -sin * 0.8) - h(-cos * 0.8, sin * 0.8)) / 1.6).atan();
            let model = body_model_matrix(p, heading, pitch, roll);
            app.mesh().draw(
                app.scene_program,
                &camera,
                &model,
                [1.0, 0.18, 0.08],
                &app.session.vehicle,
            );
        }
        let prefix = if hybrid { "projected-" } else { "" };
        crate::tests::capture_frame(&directory.join(format!("{prefix}{name}.ppm")));
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
    }
    if hybrid {
        let stats = || app.mesh().aerial.borrow().as_ref().unwrap().stats();
        let before = stats();
        let far = chase_camera(
            crate::WINDOW_WIDTH as f32 / crate::WINDOW_HEIGHT as f32,
            crate::camera::Pose {
                eye: [2500.0, 100.0, 2500.0],
                target: [2510.0, 90.0, 2510.0],
            },
        );
        app.mesh().draw_ground(app.scene_program, &far, &[]);
        assert_eq!(stats().0, 0, "distant imagery must unload");
        let near = chase_camera(
            crate::WINDOW_WIDTH as f32 / crate::WINDOW_HEIGHT as f32,
            crate::camera::Pose {
                eye: [p[0], p[1] + 4.0, p[2] - 10.0],
                target: p,
            },
        );
        app.mesh().draw_ground(app.scene_program, &near, &[]);
        assert!(
            stats().0 > 0 && stats().1 > before.1,
            "returning must reload imagery"
        );
        let reads = stats().1;
        app.mesh().draw_ground(app.scene_program, &near, &[]);
        assert_eq!(stats().1, reads, "stationary frames must reuse imagery");
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
    }
    unsafe {
        ::gl::DeleteTextures(1, &texture);
        ::gl::DeleteBuffers(1, &vbo);
        ::gl::DeleteVertexArrays(1, &vao);
        ::gl::DeleteProgram(program);
    }
}
