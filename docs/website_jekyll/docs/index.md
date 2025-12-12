---

layout: single

title: "Documentation"

permalink: /docs/

classes: wide

toc: false

---



<style>

/\* Big banner-like tiles \*/

.doc-grid {

&nbsp; display: grid;

&nbsp; gap: 1rem;

}

@media (min-width: 740px) {

&nbsp; .doc-grid { gap: 1.25rem; }

}

.doc-tile {

&nbsp; position: relative;

&nbsp; display: block;

&nbsp; height: 120px;

&nbsp; border-radius: 14px;

&nbsp; overflow: hidden;

&nbsp; text-decoration: none;

&nbsp; color: #fff;

&nbsp; box-shadow: 0 8px 24px rgba(0,0,0,.18);

&nbsp; transform: translateZ(0);

}

.doc-tile .bg {

&nbsp; position: absolute; inset: 0;

&nbsp; background-size: cover; background-position: center;

&nbsp; filter: saturate(0.9) contrast(0.95) brightness(0.9);

&nbsp; transition: transform .6s cubic-bezier(.2,.6,.2,1), filter .3s ease;

}

.doc-tile::after{

&nbsp; content:""; position:absolute; inset:0;

&nbsp; background: linear-gradient(180deg, rgba(0,0,0,.40), rgba(0,0,0,.55));

}

.doc-tile h3{

&nbsp; position: absolute; right: 1.25rem; bottom: 1rem; margin: 0;

&nbsp; font-weight: 800; font-size: clamp(1.15rem, 2.5vw, 1.6rem);

&nbsp; letter-spacing: .2px; text-shadow: 0 2px 10px rgba(0,0,0,.35);

}

.doc-tile:hover .bg { transform: scale(1.05); filter: saturate(1.1) contrast(1.05) brightness(1.0); }



/\* Gentle color wash if an image is missing \*/

.doc-tile\[data-fallback] .bg {

&nbsp; background-image: radial-gradient(1200px 500px at 20% 0%,

&nbsp;   #c9d7e8 0%, #8fb0d6 30%, #5e86b6 60%, #2c3e50 100%);

}

</style>


<div class="doc-grid">


&nbsp; <a class="doc-tile" href="{{ '/quickstart/' | relative_url }}">

&nbsp;   <div class="bg" style="background-image:url('{{ '/assets/images/docs/tiles/get-started.jpg' | relative_url }}');"></div>

&nbsp;   <h3>Get Started</h3>

&nbsp; </a>



&nbsp; <a class="doc-tile" href="{{ '/assets/bom/nml\_hand\_exo\_bom.xlsx' | relative_url }}">

&nbsp;   <div class="bg" style="background-image:url('{{ '/assets/images/docs/tiles/bom.jpg' | relative_url }}');"></div>

&nbsp;   <h3>BOM</h3>

&nbsp; </a>



&nbsp; <a class="doc-tile" href="{{ '/print/' | relative_url }}">

&nbsp;   <div class="bg" style="background-image:url('{{ '/assets/images/docs/tiles/3d-print.jpg' | relative_url }}');"></div>

&nbsp;   <h3>3D Print</h3>

&nbsp; </a>



&nbsp; <a class="doc-tile" href="{{ '/assembly/' | relative_url }}">

&nbsp;   <div class="bg" style="background-image:url('{{ '/assets/images/docs/tiles/assembly.jpg' | relative_url }}');"></div>

&nbsp;   <h3>Assembly</h3>

&nbsp; </a>



&nbsp; <a class="doc-tile" href="{{ '/electronics/' | relative_url }}">

&nbsp;   <div class="bg" style="background-image:url('{{ '/assets/images/docs/tiles/electronics.jpg' | relative_url }}');"></div>

&nbsp;   <h3>Electronics</h3>

&nbsp; </a>



&nbsp; <a class="doc-tile" href="{{ '/firmware/' | relative_url }}">

&nbsp;   <div class="bg" style="background-image:url('{{ '/assets/images/docs/tiles/firmware.jpg' | relative_url }}');"></div>

&nbsp;   <h3>Firmware</h3>

&nbsp; </a>



&nbsp; <a class="doc-tile" href="{{ '/examples/' | relative_url }}">

&nbsp;   <div class="bg" style="background-image:url('{{ '/assets/images/docs/tiles/control.jpg' | relative_url }}');"></div>

&nbsp;   <h3>Control Software</h3>

&nbsp; </a>



</div>


