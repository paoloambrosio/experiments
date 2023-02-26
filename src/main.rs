mod scene;
mod kn5_loader;
mod scene_printer;

fn main() {
    let scene = kn5_loader::load_kn5_file("abarth500_D.kn5").unwrap();
    scene_printer::print_scene(scene);
}
