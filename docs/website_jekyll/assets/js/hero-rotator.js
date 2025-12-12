// Minimal Mistakes hero background rotator (caption always on top)
(function () {
  const HERO = document.querySelector('.page__hero--overlay');
  if (!HERO) return;

  // Mount under the caption, inside the theme's image wrapper
  const WRAP = HERO.querySelector('.page__hero-image') || HERO;

  // Find images: prefer a <div class="hero-rotator" data-images="[...]">, anywhere
  let DATA_NODE =
      HERO.querySelector('.hero-rotator[data-images]') ||
      document.querySelector('.hero-rotator[data-images]');

  let IMAGES = [];
  let INTERVAL = 5000; // ms
  let FADE = 900;      // ms

  if (DATA_NODE) {
    try {
      IMAGES = JSON.parse(DATA_NODE.getAttribute('data-images') || '[]');
      INTERVAL = parseInt(DATA_NODE.getAttribute('data-interval') || '5000', 10);
      FADE = parseInt(DATA_NODE.getAttribute('data-fade') || '900', 10);
    } catch (e) { /* ignore bad JSON */ }
  } else if (Array.isArray(window.HERO_IMAGES)) {
    IMAGES = window.HERO_IMAGES;
  }

  if (!IMAGES.length) return;

  // Container for two background layers (A/B crossfade)
  const container = document.createElement('div');
  container.className = 'hero-rotator-bg';
  container.style.setProperty('--fade', FADE + 'ms');
  WRAP.appendChild(container);

  const layerA = document.createElement('div');
  const layerB = document.createElement('div');
  layerA.className = 'hr-bg hr--active';
  layerB.className = 'hr-bg';
  container.appendChild(layerA);
  container.appendChild(layerB);

  // Helpers
  const setBG = (el, src) => { el.style.backgroundImage = 'url("' + src + '")'; };

  let cur = 0;
  let next = (cur + 1) % IMAGES.length;
  let top = layerA;
  let bottom = layerB;

  // Prime first two
  setBG(top, IMAGES[cur]);
  setBG(bottom, IMAGES[next]);

  // Advance logic (preload next, then crossfade)
  function advance() {
    const preload = new Image();
    const targetIndex = (next + 1) % IMAGES.length;
    preload.src = IMAGES[next];
    // Once 'next' is guaranteed cached, swap layers
    const doSwap = () => {
      setBG(bottom, IMAGES[next]);
      bottom.classList.add('hr--active');
      top.classList.remove('hr--active');
      // swap refs
      const tmp = top; top = bottom; bottom = tmp;
      cur = next;
      next = targetIndex;
    };
    if (preload.decode) {
      preload.decode().then(doSwap).catch(doSwap);
    } else {
      preload.onload = doSwap; preload.onerror = doSwap;
    }
  }

  let timer = null;
  function play() { stop(); timer = setInterval(advance, INTERVAL); }
  function stop() { if (timer) { clearInterval(timer); timer = null; } }

  // Start
  play();

  // Optional pauses (comment out if debugging rotation)
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) stop(); else play();
  });

  if ('IntersectionObserver' in window) {
    const io = new IntersectionObserver(entries => {
      const vis = entries.some(e => e.isIntersecting);
      if (vis) play(); else stop();
    }, { threshold: 0.1 });
    io.observe(HERO);
  }
})();
