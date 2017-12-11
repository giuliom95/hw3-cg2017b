#include <thread>

#define YGL_IMAGEIO_IMPLEMENTATION 1
#include "yocto_gl.h"

using namespace ygl;
using rng_t = ygl::rng_pcg32;

constexpr const auto ray_eps = 1e-4f;

struct point {
    const instance* ist = nullptr;
    vec3f x = zero3f;
    vec3f n = zero3f;
    vec3f le = zero3f;
    vec3f o = zero3f;
    vec3f kd = zero3f;
    vec3f ks = zero3f;
    float rs = 0.5;

    bool hit() const { return (bool)ist; }
    bool emission_only() const {
        return kd == vec3f{0, 0, 0} && ks == vec3f{0, 0, 0};
    }
};

inline vec4f lookup_texture(
	const texture* txt, const vec2i& ij, bool srgb = true) {

	if (txt->ldr) {
		auto v = txt->ldr[ij];
		return (srgb) ? srgb_to_linear(v) : byte_to_float(v);
	} else if (txt->hdr) {
		return txt->hdr[ij];
	} else {
		assert(false);
		return {};
	}
}


vec4f eval_texture(
	const texture* txt, const vec2f& texcoord, bool srgb = true) {

	if (!txt) return {1, 1, 1, 1};
	auto wh = vec2i{txt->width(), txt->height()};

	auto st = vec2f{
		fmod(texcoord.x, 1.0f) * wh.x, fmod(texcoord.y, 1.0f) * wh.y};
	if (st.x < 0) st.x += wh.x;
	if (st.y < 0) st.y += wh.y;

	auto ij = clamp(vec2i{(int)st.x, (int)st.y}, {0, 0}, wh);
	auto uv = st - vec2f{(float)ij.x, (float)ij.y};

	vec2i idx[4] = {ij, {ij.x, (ij.y + 1) % wh.y},
		{(ij.x + 1) % wh.x, ij.y}, {(ij.x + 1) % wh.x, (ij.y + 1) % wh.y}};
	auto w = vec4f{(1 - uv.x) * (1 - uv.y), (1 - uv.x) * uv.y,
		uv.x * (1 - uv.y), uv.x * uv.y};

	// handle interpolation
	return (lookup_texture(txt, idx[0], srgb) * w.x +
			lookup_texture(txt, idx[1], srgb) * w.y +
			lookup_texture(txt, idx[2], srgb) * w.z +
			lookup_texture(txt, idx[3], srgb) * w.w);
}

/// Sample the camera for pixel i, j with image resolution res.
ray3f sample_camera(const camera* cam, int i, int j, int res, rng_t& rng) {
	const auto sh = res;
	const auto sw = res * cam->aspect;
	const auto u = (i + next_rand1f(rng)) / sw;
	const auto v = (j + next_rand1f(rng)) / sh;
	const auto h = 2 * std::tan(cam->yfov / 2);
    const auto w = h * cam->aspect;

	const auto ql = vec3f{-(u-0.5f)*w, (v-0.5f)*h, 1};
	const auto ol = vec3f{0, 0, 0};
	return {transform_point(cam->frame, ol),
		transform_direction(cam->frame, normalize(ol-ql))};
}

/// Evaluate the point properties for a shape.
point eval_point(const instance* ist, int ei, const vec4f& ew, const vec3f& o) {
	point p;
	p.ist = ist;
	p.o = o;

	const auto shp = ist->shp;
	const auto tri = shp->triangles[ei];
	p.x = ew.x*shp->pos[tri.x]	+ ew.y*shp->pos[tri.y]	+ ew.z*shp->pos[tri.z];
	p.n = ew.x*shp->norm[tri.x]	+ ew.y*shp->norm[tri.y]	+ ew.z*shp->norm[tri.z];

	const auto uv = ew.x*shp->texcoord[tri.x] +
					ew.y*shp->texcoord[tri.y] +
					ew.z*shp->texcoord[tri.z];

	p.kd = shp->mat->kd * eval_texture(shp->mat->kd_txt.txt, uv).xyz();
	p.le = shp->mat->ke;

	return p;
}

/// Evaluate the point properties for an environment (only o and le).
point eval_point(const environment* env, const vec3f& o) {

	//Using LatLong parametrization (slide 110)
	point p;
	const vec2f uv{
		static_cast<float>(atan2(o.z, o.x)/(2*pi)),
		static_cast<float>(acos(o.y)/pi)
	};
	p.le = env->ke * eval_texture(env->ke_txt.txt, uv).xyz();
	return p;
}

/// Intersection the scene and return a point. Support both shape and
/// environments.
point intersect(
	const scene* scn, const vec3f& q, const vec3f& i, float tmax = flt_max) {

	float t;
	int iid, eid;
	vec4f euv;

	const ray3f r{q,i,0,tmax};

	if(!intersect_ray(scn, r, false, t, iid, eid, euv)) {
		if(!scn->environments.empty()) {
			return eval_point(scn->environments.front(), -i);
		}
	} else {
		const auto ist = scn->instances[iid];
		return eval_point(ist, eid, euv, -i);
	}

	return {};
}

vec3f sample_hemisphere(const point& pt, rng_t& rng) {
	const auto frame = make_frame3_fromz(pt.x, pt.n);

	const auto r2 = next_rand1f(rng);
	const auto a = 2*pif*next_rand1f(rng);
	const auto b = sqrt(1-r2);

	const auto o = vec3f((float)(b*cos(a)), (float)(b*sin(a)), r2);

	return transform_direction(frame, o);
}


/// Naive pathtracing called recurively. Hint: call reculsively with bounces-1.
/// In this method, use hemispherical cosine sampling and only lambert BSDF.
vec3f estimate_li_naive(
	const scene* scn, const vec3f& q, const vec3f& d, int bounces, rng_t& rng) {

	auto pt = intersect(scn, q, d);
	if(pt.emission_only()) return pt.le;

	if(bounces == 0) return pt.le;

	auto i = sample_hemisphere(pt, rng);
	auto pdf = dot(pt.n, i) / pif;
	auto pr = min(1.0f, dot(pt.n, i)/pdf);
	if(next_rand1f(rng) > pr) return pt.le;

	auto li = estimate_li_naive(scn, pt.x+ray_eps*i, i, bounces-1, rng);

	auto lr = li * pt.kd * dot(pt.n, i) / (pr*pdf);
	return pt.le + lr;
}


/// Produce formulation of pathtracing that matches exactly eh above.
/// In this method, use hemispherical cosine sampling and only lambert BSDF.
vec3f estimate_li_product(
	//DOES NOT WORK
	const scene* scn, const vec3f& q, const vec3f& d, int bounces, rng_t& rng) {

	auto pt = intersect(scn, q, d);
	auto li = pt.le;
	auto w = vec3f(1,1,1);

	for(auto bounce = 0; bounce < bounces; ++bounce) {
		if(pt.emission_only()) break;

		auto i =  sample_hemisphere(pt, rng);
		auto pdf = dot(pt.n, i) / pif;
		auto pr = min(1.0f, abs(dot(pt.n, i))/pdf);
		if(next_rand1f(rng) > pr) break;

		auto bpt = intersect(scn, pt.x+ray_eps*i, i);
		w *= pt.kd;
		li += w * bpt.le;

		pt = bpt;
	}

	return li;
}

/// Initialize the lights vector and compute the shape distribution with
/// `sample_XXX_cdf`. For the homework support only point and triangles.
void init_lights(scene* scn) {
    // YOURN CODE GOES HERE
    scn->lights.clear();
}

/// Pick one light at random and sample it with area sampling.
/// Returns the point on the light source evaluate through eval_point().
/// For sampling shapes, use the `sample_XXX()` functions.
/// For the homework support only point and triangles
/// If no lights are present, just return {}.
/// To use with MIS, fold the cosine at the light and r^2 into this funciton.
point sample_lights(const scene* scn, const point& pt, rng_t& rng) {
    // YOURN CODE GOES HERE
    return {};
}

/// Compute the light sampling weight, which 1/pdf
float weight_lights(const scene* scn, const point& lpt, const point& pt) {
    // YOURN CODE GOES HERE
    return {};
}

/// Evaluate the BSDF*cosine for a triangle surface. As BSDF use Kd/pi +
/// ks*D()*G()/4cos()cos(), using the GGX for D
vec3f eval_triangle_brdfcos(const point& pt, const vec3f& i) {
    // YOURN CODE GOES HERE
    return {};
}

/// Evaluate the BSDF*cosine for a line set. Left as example.
vec3f eval_line_brdfcos(const point& pt, const vec3f& i) {
    const auto& o = pt.o;
    const auto& n = pt.n;

    auto brdfcos = vec3f{0, 0, 0};

    auto h = normalize(o + i);
    auto ndo = dot(n, o), ndi = dot(n, i), ndh = dot(h, n);

    auto so = sqrt(clamp(1 - ndo * ndo, 0.0f, 1.0f)),
         si = sqrt(clamp(1 - ndi * ndi, 0.0f, 1.0f)),
         sh = sqrt(clamp(1 - ndh * ndh, 0.0f, 1.0f));

    if (pt.kd != vec3f{0, 0, 0}) {
        auto diff = pt.kd * (si / pif);
        brdfcos += diff;
    }

    if (pt.ks != vec3f{0, 0, 0}) {
        auto ns = 2 / (pt.rs * pt.rs) - 2;
        auto d = (ns + 2) * pow(sh, ns) / (2 + pif);
        auto spec = pt.ks * (si * d / (4.0f * si * so));
        brdfcos += spec;
    }

    return brdfcos;
}

/// Evaluate the BSDF*cosine for a point set. Left as example.
vec3f eval_point_brdfcos(const point& pt, const vec3f& i) {
    const auto& o = pt.o;

    auto ido = dot(o, i);
    return (pt.kd + pt.ks) * ((2 * ido + 1) / (2 * pif));
}

/// Evaluate the BSDF*cosine for a point.
vec3f eval_brdfcos(const point& pt, const vec3f& i) {
    if (pt.emission_only()) return {0, 0, 0};
    if (!pt.ist->shp->points.empty()) {
        return eval_point_brdfcos(pt, i);
    } else if (!pt.ist->shp->lines.empty()) {
        return eval_line_brdfcos(pt, i);
    } else if (!pt.ist->shp->triangles.empty()) {
        return eval_triangle_brdfcos(pt, i);
    } else {
        return {0, 0, 0};
    }
}

/// Evaluate the BSDF*cosine as discussed in the slides
vec3f sample_triangle_brdfcos(const point& pt, rng_t& rng) {
    // YOURN CODE GOES HERE
    return {};
}

/// Comute the weight for BSDF sampling, i.e. 1 / pdf.
float weight_triangle_brdfcos(const point& pt, const vec3f& i) {
    // YOURN CODE GOES HERE
    return {};
}

vec3f sample_spherical_dir(const point& pt, rng_t& rng) {
    auto rn = vec2f{next_rand1f(rng), next_rand1f(rng)};
    auto rz = rn.y, rr = sqrtf(1 - rz * rz), rphi = 2 * pif * rn.x;
    auto wi_local = vec3f{rr * cosf(rphi), rr * sinf(rphi), rz};
    return transform_direction(make_frame3_fromz(pt.x, pt.n), wi_local);
}

float weight_spherical_dir() { return 1 / (4 * pif); }

/// Sample the BSDF*cosine
vec3f sample_brdfcos(const point& pt, rng_t& rng) {
    if (pt.emission_only()) return vec3f{0, 0, 0};
    if (!pt.ist->shp->points.empty()) {
        return sample_spherical_dir(pt, rng);
    } else if (!pt.ist->shp->lines.empty()) {
        return sample_spherical_dir(pt, rng);
    } else if (!pt.ist->shp->triangles.empty()) {
        return sample_triangle_brdfcos(pt, rng);
    } else {
        return {0, 0, 0};
    }
}

/// Weight for BSDF*cosine
float weight_brdfcos(const point& pt, const vec3f& i) {
    if (pt.emission_only()) return 0;
    if (!pt.ist->shp->points.empty()) {
        return weight_spherical_dir();
    } else if (!pt.ist->shp->lines.empty()) {
        return weight_spherical_dir();
    } else if (!pt.ist->shp->triangles.empty()) {
        return weight_triangle_brdfcos(pt, i);
    } else {
        return 0;
    }
}

/// Pathtracing with direct+indirect and russian roulette
vec3f estimate_li_direct(
    const scene* scn, const vec3f& q, const vec3f& d, int bounces, rng_t& rng) {
    // YOURN CODE GOES HERE
    return {};
}

/// Pathtracing with direct+indirect, MIS and russian roulette
vec3f estimate_li_mis(
    const scene* scn, const vec3f& q, const vec3f& d, int bounces, rng_t& rng) {
    // YOURN CODE GOES HERE
    return {};
}

image4f pathtrace(const scene* scn, int resolution, int samples,
    const std::string& integrator, int bounces, bool parallel) {
    auto cam = scn->cameras.front();
    auto img = image4f(
        (int)std::round(cam->aspect * resolution), resolution, {0, 0, 0, 0});

    auto estimate_li = estimate_li_naive;
    if (integrator == "naive") {
        estimate_li = estimate_li_naive;
    } else if (integrator == "product") {
        estimate_li = estimate_li_product;
    } else if (integrator == "direct") {
        estimate_li = estimate_li_direct;
    } else if (integrator == "mis") {
        estimate_li = estimate_li_mis;
    } else {
        throw std::runtime_error("bad integrator name");
    }

    if (!parallel) {
        for (auto j = 0; j < img.height(); j++) {
            for (auto i = 0; i < img.width(); i++) {
                auto rng = init_rng(0, (j * img.width() + i) * 2 + 1);
                img[{i, j}] = {0, 0, 0, 0};
                for (auto s = 0; s < samples; s++) {
                    auto ray = sample_camera(cam, i, j, resolution, rng);
                    auto li = estimate_li(scn, ray.o, ray.d, bounces, rng);
                    if (!isfinite(li)) continue;
                    img.at(i, j) += {li, 1};
                }
                img[{i, j}] /= (float)(samples);
            }
        }
    } else {
        auto nthreads = std::thread::hardware_concurrency();
        auto threads = std::vector<std::thread>();
        for (auto tid = 0; tid < nthreads; tid++) {
            threads.push_back(std::thread([=, &img]() {
                for (auto j = tid; j < img.height(); j += nthreads) {
                    for (auto i = 0; i < img.width(); i++) {
                        auto rng = init_rng(0, (j * img.width() + i) * 2 + 1);
                        img[{i, j}] = {0, 0, 0, 0};
                        for (auto s = 0; s < samples; s++) {
                            auto ray =
                                sample_camera(cam, i, j, resolution, rng);
                            img.at(i, j) +=
                                {estimate_li(scn, ray.o, ray.d, bounces, rng),
                                    1};
                        }
                        img[{i, j}] /= (float)(samples);
                    }
                }
            }));
        }
        for (auto& thread : threads) thread.join();
    }

    return img;
}

int main(int argc, char** argv) {
    // command line parsing
    auto parser = make_parser(argc, argv, "raytrace", "raytrace scene");
    auto parallel = parse_flag(parser, "--parallel", "-p", "runs in parallel");
    auto resolution = parse_opt<int>(
        parser, "--resolution", "-r", "vertical resolution", 720);
    auto samples =
        parse_opt<int>(parser, "--samples", "-s", "per-pixel samples", 1);
    auto bounces = parse_opt<int>(
        parser, "--bounces", "-b", "maximum number of bounces", 2);
    auto integrator =
        parse_opt<string>(parser, "--integrator", "-i", "estimation algorithm",
            "direct", false, {"naive", "product", "direct", "mis"});
    auto imageout =
        parse_opt<string>(parser, "--output", "-o", "output image", "out.png");
    auto scenein =
        parse_arg<std::string>(parser, "scenein", "input scene", "scene.obj");
    if (should_exit(parser)) {
        std::cout << get_usage(parser);
        return 1;
    }

    // load scene
    log_info("loading scene " + scenein);
    auto scn = load_scene(scenein);

    // add missing data
    auto add_opts = add_elements_options::none();
    add_opts.smooth_normals = true;
    add_opts.pointline_radius = 0.001f;
    add_opts.shape_instances = true;
    add_opts.default_camera = true;
    add_opts.default_environment = true;
    add_elements(scn, add_opts);

    // create bvh
    log_info("creating bvh");
    build_bvh(scn);

    // init lights
    init_lights(scn);

    // raytrace
    log_info("tracing scene");
    auto hdr =
        pathtrace(scn, resolution, samples, integrator, bounces, parallel);
    // tonemap and save
    log_info("saving image " + imageout);
    auto ldr = tonemap_image(hdr, tonemap_type::srgb, 0, 2.2);
    save_image4b(imageout, ldr);
}
