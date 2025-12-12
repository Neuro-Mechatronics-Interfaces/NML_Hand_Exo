# Quick Testing Guide

## Test Now (5 minutes)

### 1. Build Check
```powershell
cd "c:\Users\HP\Documents\Github\NML_Hand_Exo\exo_docs_scaffold\website_jekyll"
bundle exec jekyll build
```
**Expected:** Build completes with no errors

---

### 2. Local Preview
```powershell
bundle exec jekyll serve --livereload
```
**Expected:** Server starts at http://localhost:4000/NML_Hand_Exo/

---

### 3. Visual Checks (Open in Browser)

Visit: `http://localhost:4000/NML_Hand_Exo/`

#### Home Page:
- [ ] Hero images rotating every 4.5 seconds?
- [ ] Text fades in smoothly?
- [ ] Feature tiles lift on hover?
- [ ] Navigation has blue underline on hover?

#### Quickstart Page:
- [ ] Code blocks have copy button on hover?
- [ ] Copy button changes to "Copied!" with checkmark?
- [ ] Table of contents sticky on right side?
- [ ] Smooth scroll when clicking TOC links?

#### Assembly Page:
- [ ] Images load correctly?
- [ ] Tables display properly?
- [ ] Notice boxes (warnings) styled correctly?

#### Examples Page:
- [ ] Feature rows display?
- [ ] Cards have hover effects?
- [ ] Modals work (if present)?

#### Scroll Test:
- [ ] Back-to-top button appears after scrolling 300px?
- [ ] Clicking back-to-top smoothly scrolls to top?
- [ ] Content fades in as you scroll? (if wrapped in .fade-in-section)

#### Mobile Test (Resize Browser):
- [ ] Navigation collapses to hamburger menu?
- [ ] Cards stack vertically?
- [ ] Tables scroll horizontally if needed?
- [ ] All content readable on 375px width?

---

## What to Look For

### ✅ **Good Signs:**
- Build completes without errors
- Server starts without warnings
- All pages load quickly
- Animations are smooth (60fps)
- No console errors (F12 → Console)
- Copy buttons work on code blocks
- Navigation feels responsive
- Site looks professional

### ⚠️ **Warning Signs:**
- Build errors or warnings
- 404 errors in console
- Missing images
- Broken links
- Animations are janky
- Copy buttons don't appear
- Layout breaks on mobile

---

## Quick Fixes

### If build fails:
```powershell
bundle install
bundle exec jekyll clean
bundle exec jekyll build
```

### If custom CSS/JS not loading:
Check browser console (F12) → Network tab
- custom.css should show 200 status
- custom.js should show 200 status

### If animations don't work:
Open console (F12) → Look for JavaScript errors
Should see: "NML Hand Exo site initialized ✓"

---

## Test Results Template

Copy and fill this out:

```
## Test Results - [DATE]

### Build Test:
- Build status: [ ] Pass / [ ] Fail
- Errors: [list any]

### Visual Test:
- Hero rotation: [ ] Working / [ ] Not working
- Hover effects: [ ] Working / [ ] Not working
- Copy buttons: [ ] Working / [ ] Not working
- Back-to-top: [ ] Working / [ ] Not working
- Navigation: [ ] Working / [ ] Not working

### Mobile Test:
- 375px width: [ ] Good / [ ] Issues
- 768px width: [ ] Good / [ ] Issues
- Navigation: [ ] Good / [ ] Issues

### Performance:
- Page load: [ ] Fast / [ ] Slow
- Animations: [ ] Smooth / [ ] Janky
- Console errors: [ ] None / [ ] [list]

### Overall:
- Ready for deployment: [ ] Yes / [ ] No / [ ] Almost
- Issues to fix: [list]
```

---

## Next Steps After Testing

1. **If everything works:**
   - Read JEKYLL_REVIEW.md for polish recommendations
   - Start content review and updates
   - Optimize images
   - Prepare for GitHub Pages deployment

2. **If issues found:**
   - Note specific problems
   - Check browser console for errors
   - Review file paths and links
   - Verify all files were created/modified correctly

3. **When ready to deploy:**
   ```powershell
   git add .
   git commit -m "Production-ready Jekyll site"
   git push origin main
   ```
   Then configure GitHub Pages in repo settings.

---

## Contact

If you encounter issues not covered here:
- Check JEKYLL_REVIEW.md (comprehensive troubleshooting)
- Check IMPLEMENTATION_SUMMARY.md (detailed changes)
- Review Jekyll documentation
- Check browser console for specific errors

---

**Ready?** Open PowerShell and run:
```powershell
cd "c:\Users\HP\Documents\Github\NML_Hand_Exo\exo_docs_scaffold\website_jekyll"
bundle exec jekyll serve --livereload
```

Then open: http://localhost:4000/NML_Hand_Exo/
