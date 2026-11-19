// Corrugated cartons 180 x 250 x 120 mm, 3 kg: top flaps closed with tape, shipping label with a barcode, handling marks.
// Origin at the bottom-face centre. All artwork is generic (no brands).
function cartonLabelTex(k) {
  const c = document.createElement("canvas"); c.width = 256; c.height = 160;
  const g = c.getContext("2d");
  g.fillStyle = "#f6f4ee"; g.fillRect(0, 0, 256, 160);
  g.fillStyle = "#1a1a1a"; g.font = "700 26px 'IBM Plex Mono', monospace"; g.fillText(`SKU 40-${String(1200 + k).padStart(4, "0")}`, 12, 32);
  g.font = "500 18px 'IBM Plex Mono', monospace"; g.fillText(`NET 3.0 kg · LOT 2611-${String(k + 1).padStart(2, "0")}`, 12, 56);
  let x = 14, seed = 9301 + k * 49297;                         // pseudo-random barcode bars (deterministic)
  while (x < 242) { seed = (seed * 233280 + 49297) % 233280; const w = 1 + (seed % 4); if (seed % 3) g.fillRect(x, 70, w, 62); x += w + 1; }
  g.font = "500 14px 'IBM Plex Mono', monospace"; g.fillText(`4 006381 ${String(333931 + k * 7).padStart(6, "0")}`, 40, 150);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 4; return t;
}
const marksTex = (() => {
  const c = document.createElement("canvas"); c.width = 128; c.height = 64;
  const g = c.getContext("2d"); g.clearRect(0, 0, 128, 64); g.strokeStyle = g.fillStyle = "#2a1d12"; g.lineWidth = 5;
  for (const x of [34, 64]) { g.beginPath(); g.moveTo(x, 54); g.lineTo(x, 18); g.stroke(); g.beginPath(); g.moveTo(x - 12, 26); g.lineTo(x, 10); g.lineTo(x + 12, 26); g.fill(); }
  g.beginPath(); g.arc(102, 34, 14, Math.PI * 0.15, Math.PI * 0.85, true); g.stroke(); g.fillRect(100, 18, 4, 30);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
})();
const marksMat = new THREE.MeshStandardMaterial({ map: marksTex, transparent: true, roughness: 0.9, depthWrite: false });
const cartonEdgeMat = phys(0xa77f55, 0.9, 0, { map: M.carton.map });
function buildCarton(k) {
  const g = new THREE.Group(); world.add(g);
  const { x, y, h } = CARTON;
  rbox(x, y, h, 0.0035, M.carton, 0, 0, h / 2, g);
  // top flaps: two slightly raised panels with a seam between them, tape over the seam and down the ends
  for (const s of [-1, 1]) rbox(x / 2 - 0.002, y - 0.004, 0.0015, 0.0006, cartonEdgeMat, s * (x / 4), 0, h + 0.0004, g);
  rbox(0.048, y + 0.003, 0.0012, 0.0005, M.cartonTape, 0, 0, h + 0.0016, g);
  for (const s of [-1, 1]) rbox(0.048, 0.0012, 0.055, 0.0005, M.cartonTape, 0, s * (y / 2 + 0.0008), h - 0.026, g);
  // shipping label on one long side, handling marks on the other
  const lab = new THREE.Mesh(new THREE.PlaneGeometry(0.10, 0.0625), new THREE.MeshStandardMaterial({ map: cartonLabelTex(k), roughness: 0.6 }));
  lab.rotation.set(Math.PI / 2, Math.PI / 2, 0); lab.position.set(x / 2 + 0.0009, 0.04, h * 0.5); lab.receiveShadow = true; g.add(lab);
  const mk = new THREE.Mesh(new THREE.PlaneGeometry(0.07, 0.035), marksMat);
  mk.rotation.set(Math.PI / 2, -Math.PI / 2, 0); mk.position.set(-x / 2 - 0.0009, -0.05, h * 0.62); g.add(mk);
  g.visible = false;
  return g;
}
const cartonMeshes = cartons.map((c) => buildCarton(c.k));
function placeCartons(t) {
  let queue = 0, done = 0;
  cartons.forEach((c, i) => {
    const p = cartonAt(c, t), o = cartonMeshes[i];
    o.visible = !!p;
    if (p) o.position.set(p[0], p[1], p[2]);
    if (c.place != null && t >= c.place) done++;
    else if (p && (c.pick == null || t < c.pick)) queue++;
  });
  return { queue, done };
}
