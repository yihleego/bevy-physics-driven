use bevy::prelude::*;
#[cfg(not(target_arch = "wasm32"))]
use bevy::sprite_render::{Wireframe2dConfig, Wireframe2dPlugin};
pub struct Debug2dPlugin;

impl Plugin for Debug2dPlugin {
    fn build(&self, app: &mut App) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            app.add_plugins(Wireframe2dPlugin::default());
            app.add_systems(Update, toggle_wireframe);
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn toggle_wireframe(
    mut wireframe_config: ResMut<Wireframe2dConfig>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        wireframe_config.global = !wireframe_config.global;
    }
}
