// Custom JavaScript for NML Hand Exo site
// Hero image rotation, scroll animations, and interactive effects

(function() {
  'use strict';

  // ==========================================================================
  // Hero Image Rotator
  // ==========================================================================
  
  function initHeroRotator() {
    const rotator = document.querySelector('.hero-rotator');
    if (!rotator) return;

    try {
      const config = JSON.parse(rotator.dataset.rotation || '{}');
      const images = config.images || [];
      const interval = config.interval || 4500;
      const fade = config.fade || 900;

      if (images.length === 0) return;

      let currentIndex = 0;
      const imgElements = [];

      // Create image elements
      images.forEach((src, index) => {
        const img = document.createElement('img');
        img.src = src;
        img.alt = `Hero image ${index + 1}`;
        img.style.position = 'absolute';
        img.style.top = '0';
        img.style.left = '0';
        img.style.width = '100%';
        img.style.height = '100%';
        img.style.objectFit = 'cover';
        img.style.opacity = index === 0 ? '1' : '0';
        img.style.transition = `opacity ${fade}ms cubic-bezier(0.4, 0, 0.2, 1)`;
        rotator.appendChild(img);
        imgElements.push(img);
      });

      // Rotation function
      function rotateImages() {
        const prevIndex = currentIndex;
        currentIndex = (currentIndex + 1) % images.length;

        imgElements[prevIndex].style.opacity = '0';
        imgElements[currentIndex].style.opacity = '1';
      }

      // Start rotation
      if (images.length > 1) {
        setInterval(rotateImages, interval);
      }
    } catch (e) {
      console.error('Hero rotator error:', e);
    }
  }

  // ==========================================================================
  // Scroll Animations (Intersection Observer)
  // ==========================================================================
  
  function initScrollAnimations() {
    const sections = document.querySelectorAll('.fade-in-section');
    if (sections.length === 0) return;

    const observerOptions = {
      threshold: 0.15,
      rootMargin: '0px 0px -50px 0px'
    };

    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          entry.target.classList.add('is-visible');
          // Optional: stop observing after animation
          // observer.unobserve(entry.target);
        }
      });
    }, observerOptions);

    sections.forEach(section => {
      observer.observe(section);
    });
  }

  // ==========================================================================
  // Smooth Scroll for Anchor Links
  // ==========================================================================
  
  function initSmoothScroll() {
    document.querySelectorAll('a[href^="#"]').forEach(anchor => {
      anchor.addEventListener('click', function (e) {
        const href = this.getAttribute('href');
        if (href === '#' || href === '#!') return;

        const target = document.querySelector(href);
        if (!target) return;

        e.preventDefault();
        const offsetTop = target.getBoundingClientRect().top + window.pageYOffset - 80;
        
        window.scrollTo({
          top: offsetTop,
          behavior: 'smooth'
        });
      });
    });
  }

  // ==========================================================================
  // Copy Code Button
  // ==========================================================================
  
  function initCopyButtons() {
    document.querySelectorAll('.highlight pre').forEach(pre => {
      // Skip if button already exists
      if (pre.querySelector('.copy-button')) return;

      const button = document.createElement('button');
      button.className = 'copy-button';
      button.innerHTML = `
        <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
          <path d="M4 2a2 2 0 0 1 2-2h8a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H6a2 2 0 0 1-2-2V2z"/>
          <path d="M2 6a2 2 0 0 0-2 2v6a2 2 0 0 0 2 2h6a2 2 0 0 0 2-2v-2H6a3 3 0 0 1-3-3V6H2z"/>
        </svg>
        <span>Copy</span>
      `;
      button.style.cssText = `
        position: absolute;
        top: 8px;
        right: 8px;
        padding: 6px 12px;
        background: rgba(14, 99, 156, 0.95);
        color: white;
        border: 1px solid rgba(0, 122, 204, 0.8);
        border-radius: 3px;
        cursor: pointer;
        font-size: 12px;
        display: flex;
        align-items: center;
        gap: 6px;
        transition: all 0.2s ease;
        opacity: 0;
        z-index: 10;
      `;

      pre.style.position = 'relative';
      pre.appendChild(button);

      // Show on hover
      pre.addEventListener('mouseenter', () => {
        button.style.opacity = '1';
      });
      pre.addEventListener('mouseleave', () => {
        button.style.opacity = '0';
      });

      // Copy functionality
      button.addEventListener('click', async () => {
        const code = pre.querySelector('code');
        if (!code) return;

        try {
          await navigator.clipboard.writeText(code.textContent);
          button.innerHTML = `
            <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
              <path d="M13.78 4.22a.75.75 0 010 1.06l-7.25 7.25a.75.75 0 01-1.06 0L2.22 9.28a.75.75 0 011.06-1.06L6 10.94l6.72-6.72a.75.75 0 011.06 0z"/>
            </svg>
            <span>Copied!</span>
          `;
          button.style.background = 'rgba(78, 201, 176, 0.95)';
          button.style.borderColor = 'rgba(78, 201, 176, 0.8)';
          
          setTimeout(() => {
            button.innerHTML = `
              <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
                <path d="M4 2a2 2 0 0 1 2-2h8a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H6a2 2 0 0 1-2-2V2z"/>
                <path d="M2 6a2 2 0 0 0-2 2v6a2 2 0 0 0 2 2h6a2 2 0 0 0 2-2v-2H6a3 3 0 0 1-3-3V6H2z"/>
              </svg>
              <span>Copy</span>
            `;
            button.style.background = 'rgba(14, 99, 156, 0.95)';
            button.style.borderColor = 'rgba(0, 122, 204, 0.8)';
          }, 2000);
        } catch (err) {
          console.error('Copy failed:', err);
        }
      });
    });
  }

  // ==========================================================================
  // Modal Enhancements
  // ==========================================================================
  
  function initModals() {
    document.querySelectorAll('.modal-trigger').forEach(trigger => {
      trigger.addEventListener('click', (e) => {
        e.preventDefault();
        const modalId = trigger.dataset.modal;
        const modal = document.getElementById(modalId);
        if (modal) {
          modal.style.display = 'block';
          document.body.style.overflow = 'hidden';
        }
      });
    });

    // Close modals
    document.querySelectorAll('.modal-close, .modal').forEach(el => {
      el.addEventListener('click', (e) => {
        if (e.target === el || el.classList.contains('modal-close')) {
          const modal = el.classList.contains('modal') ? el : el.closest('.modal');
          if (modal) {
            modal.style.display = 'none';
            document.body.style.overflow = '';
          }
        }
      });
    });

    // ESC key to close
    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape') {
        document.querySelectorAll('.modal').forEach(modal => {
          if (modal.style.display === 'block') {
            modal.style.display = 'none';
            document.body.style.overflow = '';
          }
        });
      }
    });
  }

  // ==========================================================================
  // Particle Background (Optional)
  // ==========================================================================
  
  function initParticles() {
    const container = document.querySelector('.particles-bg');
    if (!container) return;

    const particleCount = 20;
    
    for (let i = 0; i < particleCount; i++) {
      const particle = document.createElement('div');
      particle.className = 'particle';
      
      const size = Math.random() * 60 + 20;
      particle.style.width = `${size}px`;
      particle.style.height = `${size}px`;
      particle.style.left = `${Math.random() * 100}%`;
      particle.style.top = `${Math.random() * 100}%`;
      particle.style.animationDelay = `${Math.random() * 20}s`;
      particle.style.animationDuration = `${Math.random() * 10 + 15}s`;
      
      container.appendChild(particle);
    }
  }

  // ==========================================================================
  // Back to Top Button
  // ==========================================================================
  
  function initBackToTop() {
    const button = document.createElement('button');
    button.className = 'back-to-top';
    button.innerHTML = '↑';
    button.setAttribute('aria-label', 'Back to top');
    button.style.cssText = `
      position: fixed;
      bottom: 30px;
      right: 30px;
      width: 50px;
      height: 50px;
      border-radius: 50%;
      background: #1e88e5;
      color: white;
      border: none;
      font-size: 24px;
      cursor: pointer;
      box-shadow: 0 4px 12px rgba(86, 156, 214, 0.4);
      opacity: 0;
      visibility: hidden;
      transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
      z-index: 1000;
    `;

    document.body.appendChild(button);

    // Show/hide based on scroll
    window.addEventListener('scroll', () => {
      if (window.pageYOffset > 300) {
        button.style.opacity = '1';
        button.style.visibility = 'visible';
      } else {
        button.style.opacity = '0';
        button.style.visibility = 'hidden';
      }
    });

    // Scroll to top
    button.addEventListener('click', () => {
      window.scrollTo({
        top: 0,
        behavior: 'smooth'
      });
    });

    // Hover effect
    button.addEventListener('mouseenter', () => {
      button.style.transform = 'scale(1.1)';
    });
    button.addEventListener('mouseleave', () => {
      button.style.transform = 'scale(1)';
    });
  }

  // ==========================================================================
  // Initialize Everything
  // ==========================================================================
  
  function init() {
    // Wait for DOM to be ready
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', init);
      return;
    }

    initHeroRotator();
    initScrollAnimations();
    initSmoothScroll();
    initCopyButtons();
    initModals();
    initParticles();
    initBackToTop();

    console.log('NML Hand Exo site initialized ✓');
  }

  init();
})();
