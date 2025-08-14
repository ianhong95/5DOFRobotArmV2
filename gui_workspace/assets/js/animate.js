function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene,camera);
}

animate();