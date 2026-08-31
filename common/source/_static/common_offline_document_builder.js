/*
 * [copywiki destination="copter,plane,rover,sub,blimp,antennatracker,dev,planner,planner2,ardupilot,mavproxy"]
 *
 * Generates the single-file offline export. The file is one HTML document:
 * a shell (the theme's layout, a sidebar and a search box), every page stored
 * as an inert <script type="text/plain"> block, every image stored once, and
 * an embedded script (SHELL_JS) that routes between pages by URL hash, builds
 * the sidebar tree from the pages' own toctrees and searches Sphinx's index.
 * common_offline_export.js reads the cache and streams what this produces.
 * SHELL_JS is assembled from single-quoted literals: double every backslash.
 */
(function (global) {
  'use strict';

  /* ------------------------------------------------------------ the shell */

  // Only what the app needs; the theme's stylesheet is embedded at export time.
  var SHELL_CSS =
    'html,body{height:100%}' +
    '.wy-nav-side{overflow-y:auto}' +
    '#ap-search{margin:12px;padding:8px 10px;border:0;border-radius:3px;' +
    'font:inherit;width:calc(100% - 24px)}' +
    '#ap-miss{display:none;padding:10px 16px;color:#a8620f;background:#ffedcc;' +
    'font-size:13px}' +
    '#ap-bar{background:#2980b9;color:#fff;padding:8px 16px;font-size:13px}' +
    '#ap-brand{background:#2980b9;color:#fff;padding:14px 16px;font-weight:700}' +
    '#ap-brand small{display:block;font-weight:400;opacity:.85;font-size:12px}' +
    '#ap-crumb{padding:6px 0;color:#666;font-size:13px;text-transform:uppercase;' +
    'letter-spacing:.05em}' +
    // The gap the theme's footer block leaves before the buttons.
    '#ap-foot{margin-top:24px}' +
    '#ap-lightbox{position:fixed;top:0;right:0;bottom:0;left:0;z-index:9999;' +
    'display:none;align-items:center;justify-content:center;cursor:zoom-out;' +
    'background:rgba(0,0,0,.85)}' +
    '#ap-lightbox img{max-width:94vw;max-height:94vh}' +
    '#ap-pick{list-style:none;margin:1em 0 0;padding:0}' +
    '#ap-pick li{border-bottom:1px solid #e1e4e5}' +
    '#ap-pick a{display:flex;align-items:baseline;justify-content:space-between;' +
    'gap:1em;padding:12px 2px;text-decoration:none}' +
    '#ap-pick small{color:#666;text-transform:none;font-size:.85em}' +
    '.ap-results li{padding:10px 0}' +
    '.ap-results a{padding:0;border:0}' +
    '.ap-snip{margin:.25em 0 0;color:#4a4a4a;font-size:.9em;line-height:1.5}' +
    '.ap-snip mark{background:#fff3b0;color:inherit;padding:0 2px}' +
    '.ap-actions{display:flex;flex-wrap:wrap;gap:0 1.5em}' +
    '#ap-toast{position:fixed;left:50%;bottom:24px;transform:translateX(-50%);' +
    'z-index:10000;display:none;align-items:center;gap:1em;' +
    'max-width:min(680px,92vw);padding:12px 16px;border-radius:4px;' +
    'background:#1f2d3a;color:#fff;font-size:14px;line-height:1.4;' +
    'box-shadow:0 6px 24px rgba(0,0,0,.3)}' +
    '#ap-toast.on{display:flex}' +
    '#ap-toast a{color:#8ecbff;text-decoration:underline;white-space:nowrap}' +
    '#ap-toast button{background:none;border:0;color:#c3ccd5;font:inherit;' +
    'cursor:pointer;padding:0;white-space:nowrap}';

  var SHELL_JS = [
    '(function(){',
    'var D=JSON.parse(document.getElementById("ap-index").textContent);',
    'var doc=document.getElementById("ap-doc");',
    'var nav=document.getElementById("ap-nav");',
    'var foot=document.getElementById("ap-foot");',
    'var crumb=document.getElementById("ap-crumb");',
    'var miss=document.getElementById("ap-miss");',
    'var search=document.getElementById("ap-search");',
    // Anchors are page paths, not ordinals, so bookmarks survive re-exports.
    'var byPath={};D.pages.forEach(function(p,i){byPath[p.p]=i;});',
    // Position in the toctree, for the footer buttons.
    'var orderAt={};(D.order||[]).forEach(function(p,n){',
    'if(orderAt[p]===undefined)orderAt[p]=n;});',
    'nav.innerHTML=D.nav;',
    'var links=[].slice.call(nav.querySelectorAll("a[href^=\\"#\\"]"));',
    'function current(){return (location.hash||"").replace(/^#/,"");}',
    // Accept #/rover, a trailing slash, a leftover .html, a missing slash.
    'function lookup(raw){',
    'if(!raw)return undefined;',
    'var cands=[],p=raw;',
    'if(p.charAt(0)!=="/")p="/"+p;',
    'p=p.replace(/\\.html?$/,"");',
    'cands.push(p);',
    'if(p.slice(-1)==="/")cands.push(p.slice(0,-1));',
    'var base=p.replace(/\\/$/,"");',
    'cands.push(base+"/index");',
    'cands.push(base+"/docs/index");',
    'for(var i=0;i<cands.length;i++){',
    'if(byPath[cands[i]]!==undefined)return cands[i];}',
    // Last resort: the first page under that prefix, so #/rover/docs works.
    'for(var j=0;j<D.pages.length;j++){',
    'if(D.pages[j].p.indexOf(base+"/")===0)return D.pages[j].p;}',
    'return undefined;}',

    /* ----------------------------------------- the sidebar, as the theme has it */
    // A branch is open when its <li> is "current".
    'function ancestors(el){var out=[];',
    'while(el&&el!==nav){if(el.tagName==="LI")out.push(el);el=el.parentNode;}',
    'return out;}',
    'function clearCurrent(){',
    '[].forEach.call(nav.querySelectorAll(".current"),function(el){',
    'el.classList.remove("current");',
    'if(el.tagName==="LI")el.setAttribute("aria-expanded","false");});}',
    // Open the path to the page being read and close everything else.
    'function setCurrent(path){',
    'clearCurrent();',
    'var hit=null;',
    'links.forEach(function(a){',
    'if(!hit&&a.getAttribute("href")==="#"+path)hit=a;});',
    'if(!hit)return;',
    'hit.classList.add("current");',
    'ancestors(hit).forEach(function(li){li.classList.add("current");',
    'li.setAttribute("aria-expanded","true");});',
    'if(hit.scrollIntoView)hit.scrollIntoView({block:"nearest"});}',
    // The theme's toggleCurrent: siblings close when one opens.
    'function toggleBranch(btn){',
    'var li=btn.closest?btn.closest("li"):null;',
    'if(!li||!li.parentNode)return;',
    '[].forEach.call(li.parentNode.children,function(s){',
    'if(s===li)return;',
    's.classList.remove("current");s.setAttribute("aria-expanded","false");',
    '[].forEach.call(s.querySelectorAll("li.current"),function(x){',
    'x.classList.remove("current");x.setAttribute("aria-expanded","false");});});',
    'var kids=li.querySelectorAll("ul li");',
    'if(!kids.length)return;',
    '[].forEach.call(kids,function(x){x.classList.remove("current");',
    'x.setAttribute("aria-expanded","false");});',
    'var open=li.classList.toggle("current");',
    'li.setAttribute("aria-expanded",open?"true":"false");}',

    /* ------------------------------------------------ next / previous buttons */
    'function clearFooter(){if(foot)foot.innerHTML="";}',
    // Skip pages not in the file, and stop at the wiki boundary.
    'function nearby(i,dir,w){',
    'for(var j=i+dir;j>=0&&j<D.order.length;j+=dir){',
    'var p=D.order[j];',
    'if(p.split("/")[1]!==w)return null;',
    'if(byPath[p]!==undefined)return p;}',
    'return null;}',
    'function navButton(p,dir){',
    'var t=byPath[p]!==undefined?D.pages[byPath[p]].t:p;',
    'if(dir<0)return \'<a href="#\'+esc(p)+\'" class="btn btn-neutral float-left" title="\'',
    '+esc(t)+\'" accesskey="p" rel="prev">\'',
    '+\'<span class="fa fa-arrow-circle-left" aria-hidden="true"></span> Previous</a>\';',
    'return \'<a href="#\'+esc(p)+\'" class="btn btn-neutral float-right" title="\'',
    '+esc(t)+\'" accesskey="n" rel="next">Next \'',
    '+\'<span class="fa fa-arrow-circle-right" aria-hidden="true"></span></a>\';}',
    'function showFooter(path){',
    'if(!foot)return;',
    'var i=orderAt[path];',
    'if(i===undefined){clearFooter();return;}',
    'var w=path.split("/")[1];',
    'var pv=nearby(i,-1,w),nx=nearby(i,1,w);',
    'foot.innerHTML=(pv||nx)?\'<div class="rst-footer-buttons" role="navigation"\'',
    '+\' aria-label="Footer">\'+(pv?navButton(pv,-1):"")+(nx?navButton(nx,1):"")',
    '+\'</div>\':"";}',

    /* ---------------------------------------- the parameter version switcher */
    // Fill the theme's own <select> from what this file holds.
    'function fillVersions(path){',
    'var sel=doc.querySelector("#selectPicker");if(!sel)return;',
    'var box=sel.parentNode;',
    'var wik=path.split("/")[1];',
    'var list=(D.params||{})[wik]||[];',
    'if(!list.length){if(box)box.style.display="none";return;}',
    'sel.innerHTML="";',
    // The unversioned page leads the list, so there is a way back to it.
    'var latest="/"+wik+"/docs/parameters";',
    'if(byPath[latest]!==undefined){',
    'var lo=document.createElement("option");',
    'lo.value=latest;lo.textContent="Latest";',
    'if(path===latest)lo.selected=true;',
    'sel.appendChild(lo);}',
    'list.forEach(function(v){',
    'var o=document.createElement("option");',
    'o.value=v.p;o.textContent=v.n;',
    'if(v.p===path)o.selected=true;',
    'sel.appendChild(o);});',
    'sel.addEventListener("change",function(){go(sel.value);});}',

    // Landing page when the file holds more than one wiki.
    'function showPicker(){',
    'miss.style.display="none";',
    'var rows=D.homes.map(function(h){',
    'return \'<li><a href="#\'+esc(h.path)+\'"><span>\'+esc(h.name||h.id)+\'</span>\'',
    '+\'<small>\'+esc(h.pages)+\' pages</small></a></li>\';}).join("");',
    'doc.innerHTML="<h1>Offline copy</h1><p>This file contains "+D.homes.length',
    '+" wikis. Choose one to start reading.</p><ul id=\\"ap-pick\\">"+rows+"</ul>";',
    'crumb.textContent="";',
    'document.title="ArduPilot (offline)";',
    'clearCurrent();clearFooter();',
    'var sc=document.querySelector(".wy-nav-content-wrap");if(sc)sc.scrollTop=0;}',

    // A page rather than a vanishing banner, with a way on.
    'function showMissing(raw){',
    'miss.style.display="none";',
    'var held=D.homes.map(function(h){return h.name||h.id;}).join(", ");',
    'var live="https://ardupilot.org"+(raw==="/"?"":raw+".html");',
    'doc.innerHTML="<h1>Not in this offline copy</h1>"',
    '+"<p><code>"+esc(raw)+"</code> is not included in this download.</p>"',
    '+"<p>This file contains: "+esc(held)+".</p>"',
    '+"<p class=\\"ap-actions\\">"',
    // data-ap-external keeps the click handler off it, or it loops back here.
    '+"<a href=\\""+esc(live)+"\\" data-ap-external=\\"1\\">Open it on ardupilot.org</a>"',
    '+"<a href=\\"#\\" id=\\"ap-back\\">Go back</a>"',
    '+(D.homes.length>1?"<a href=\\"#/\\">Choose a wiki</a>":"")+"</p>"',
    '+"<p><small>Opening the live wiki needs a connection. '
      + 'With none it will simply fail to load, and this file is still here.'
      + '</small></p>";',
    'var b=document.getElementById("ap-back");',
    'if(b)b.addEventListener("click",function(ev){ev.preventDefault();history.back();});',
    'crumb.textContent="";',
    'document.title="Not in this offline copy - ArduPilot";',
    'clearFooter();',
    'var sc=document.querySelector(".wy-nav-content-wrap");if(sc)sc.scrollTop=0;}',

    // Undo the </script> escaping the page blocks needed.
    'function unblock(s){return s.replace(/<\\\\\\/(script)/gi,"<\\/$1");}',

    'function show(raw){',
    'var path=lookup(raw);',
    'if(path===undefined){return showMissing(raw);}',
    'var i=byPath[path];',
    'miss.style.display="none";',
    'var el=document.getElementById("p"+i);if(!el)return;',
    'doc.innerHTML=unblock(el.textContent);',
    // Attach images only for the page being shown, so nothing else decodes.
    '[].forEach.call(doc.querySelectorAll("[data-ap-img]"),function(im){',
    'var b=document.getElementById("i"+im.getAttribute("data-ap-img"));',
    'if(b)im.src=b.textContent;});',
    'fillVersions(path);',
    'var sc=document.querySelector(".wy-nav-content-wrap");if(sc)sc.scrollTop=0;',
    // The page's own <h1> follows, so name the wiki rather than repeat it.
    'var wid=D.pages[i].p.split("/")[1]||"";',
    'var wh=null;D.homes.forEach(function(h){if(h.id===wid)wh=h;});',
    'crumb.textContent=wh?wh.name:wid.replace(/^./,function(c){return c.toUpperCase();});',
    'document.title=D.pages[i].t+" - ArduPilot (offline)";',
    'setCurrent(path);',
    'showFooter(path);}',
    'function route(){',
    'var raw=current();',
    'if(!raw||raw==="/"){',
    'return D.home?show(D.home):showPicker();}',
    // Not a path: an in-page anchor reached the hash; scroll, keep the page.
    'if(raw.charAt(0)!=="/"){',
    'var at=document.getElementById(raw);',
    'if(at&&at.scrollIntoView)at.scrollIntoView();',
    'return;}',
    'show(raw);}',
    'window.addEventListener("hashchange",route);',
    // A browser will not navigate to a data: URL, so linked images open here.
    'function lightbox(uri){',
    'var lb=document.getElementById("ap-lightbox");',
    'if(!lb){lb=document.createElement("div");lb.id="ap-lightbox";',
    'lb.addEventListener("click",function(){lb.style.display="none";});',
    'document.body.appendChild(lb);}',
    'lb.innerHTML="";',
    'var im=document.createElement("img");im.src=uri;lb.appendChild(im);',
    'lb.style.display="flex";}',
    // An anchor parses the host; a regex here would need doubled backslashes.
    'function hostOf(u){var a=document.createElement("a");a.href=u;',
    'return a.hostname||u;}',
    // A link to another host leaves the file: say so and let the reader choose.
    'var toastTimer=null;',
    'function toast(href){',
    'var t=document.getElementById("ap-toast");',
    'if(!t){t=document.createElement("div");t.id="ap-toast";',
    'document.body.appendChild(t);}',
    'clearTimeout(toastTimer);',
    't.innerHTML="";',
    'var msg=document.createElement("span");',
    'msg.textContent=hostOf(href)+" is not part of this offline copy. "',
    '+"Opening it leaves this file and needs a connection.";',
    'var go=document.createElement("a");',
    'go.href=href;go.target="_blank";go.rel="noopener";',
    'go.textContent="Open anyway";',
    'go.addEventListener("click",function(){t.className="";});',
    'var hide=document.createElement("button");',
    'hide.type="button";hide.textContent="Dismiss";',
    'hide.addEventListener("click",function(){t.className="";});',
    't.appendChild(msg);t.appendChild(go);t.appendChild(hide);',
    't.className="on";',
    'toastTimer=setTimeout(function(){t.className="";},9000);}',
    // Resolve relative content links against the current page and route them.
    'function resolve(base,href){',
    'var parts=base.split("/");parts.pop();',
    'href.split("/").forEach(function(seg){',
    'if(seg===".."){parts.pop();}else if(seg!=="."&&seg!==""){parts.push(seg);}});',
    'return parts.join("/").replace(/\\.html?$/,"");}',
    // Absolute ardupilot.org links route into the file when the target is here.
    'function siteHref(href){',
    'var m=/^https?:\\/\\/(?:www\\.)?ardupilot\\.org(\\/.*)?$/i.exec(href);',
    'if(!m)return null;',
    'var rest=(m[1]||"").replace(/[?#].*$/,"");',
    'if(!rest||rest==="/")return "/";',
    'return rest.replace(/\\.html?$/,"");}',
    // An unchanged hash fires no hashchange.
    'function go(p){var h="#"+p;if(location.hash===h){route();}else{location.hash=h;}}',
    'function onLinkClick(e){',
    // The expand arrows sit inside the anchor, so handle them first.
    'var xb=e.target.closest?e.target.closest("button.toctree-expand"):null;',
    'if(xb){e.preventDefault();e.stopPropagation();toggleBranch(xb);return;}',
    'var a=e.target.closest?e.target.closest("a[href]"):null;if(!a)return;',
    'var href=a.getAttribute("href");',
    'if(a.getAttribute("data-ap-external")!==null)return;',
    'if(!href||/^mailto:/.test(href))return;',
    // A bare fragment is an in-page anchor: scroll to it, never route.
    'if(href.charAt(0)==="#"){',
    'e.preventDefault();',
    'var fid=href.slice(1);',
    'if(fid){var ft=document.getElementById(fid)||',
    'doc.querySelector(\'[id="\'+fid.replace(/"/g,"")+\'"]\');',
    'if(ft&&ft.scrollIntoView)ft.scrollIntoView();}',
    'return;}',
    'if(/^https?:/i.test(href)){',
    'var mapped=siteHref(href);',
    // Another host: not wiki content, so no offline copy to route to.
    'if(mapped===null){e.preventDefault();toast(a.href);return;}',
    // Wiki content never leaves the file; say the wiki is missing instead.
    'e.preventDefault();',
    'if(mapped==="/"){go("/");return;}',
    'var hit=lookup(mapped);',
    'go(hit!==undefined?hit:mapped);',
    'return;}',
    'var frag="";var h=href;var hi=h.indexOf("#");',
    'if(hi>=0){frag=h.slice(hi);h=h.slice(0,hi);}',
    // Root-relative is already a site path; only relative links resolve
    // against the page being read.
    'var target=h.charAt(0)==="/"?h.replace(/\\.html?$/,""):resolve(current(),h);',
    // A relative link must never leave the file.
    'e.preventDefault();',
    'var found=lookup(target);',
    'if(found!==undefined){go(target);',
    'if(frag){setTimeout(function(){var t=doc.querySelector(frag);',
    'if(t&&t.scrollIntoView)t.scrollIntoView();},50);}return;}',
    // Thumbnails link their full-size file; answer from the index.
    'var iid=D.imgs?D.imgs[target]:undefined;',
    'if(iid!==undefined&&iid!==null){',
    'var blk=document.getElementById("i"+iid);',
    'if(blk){lightbox(blk.textContent);return;}}',
    // A scaled image links an original no page displays; show the thumbnail.
    'var inner=a.querySelector?a.querySelector("[data-ap-img]"):null;',
    'if(inner){',
    'var ib=document.getElementById("i"+inner.getAttribute("data-ap-img"));',
    'if(ib){lightbox(ib.textContent);return;}}',
    'showMissing(target);',
    '}',
    // The sidebar carries the absolute links, so it needs this too.
    'doc.addEventListener("click",onLinkClick);',
    'nav.addEventListener("click",onLinkClick);',
    'if(foot)foot.addEventListener("click",onLinkClick);',
    // Search every page by title and path; the sidebar lists only the toctree.
    'function esc(s){return String(s).replace(/[&<>"]/g,function(c){',
    'return {"&":"&amp;","<":"&lt;",">":"&gt;","\\"":"&quot;"}[c];});}',
    'function wikiName(p){var id=p.split("/")[1]||"";var out=id;',
    'D.homes.forEach(function(h){if(h.id===id)out=h.name||h.id;});return out;}',
    // Parsed on first search, not on load.
    'var SI=null;',
    'function searchIndex(){',
    'if(SI!==null)return SI;',
    'var el=document.getElementById("ap-fts");',
    'try{SI=el?JSON.parse(el.textContent):{};}catch(e){SI={};}',
    'return SI;}',
    // The index is stemmed, so stem the query with Sphinx's own stemmer.
    'var stemmer=(typeof Stemmer!=="undefined")?new Stemmer():null;',
    'function stem(w){return stemmer?stemmer.stemWord(w):w;}',
    // Sphinx drops stopwords from the index, so they must not be required.
    'var STOP=(typeof stopwords!=="undefined")?stopwords:[];',
    // Edit distance with a budget; most candidates fail the length check first.
    'function within(a,b,max){',
    'if(Math.abs(a.length-b.length)>max)return false;',
    'var prev=[],i,j;for(j=0;j<=b.length;j++)prev[j]=j;',
    'for(i=1;i<=a.length;i++){',
    'var best=i,diag=prev[0];prev[0]=i;',
    'for(j=1;j<=b.length;j++){',
    'var cur=Math.min(prev[j]+1,prev[j-1]+1,diag+(a.charAt(i-1)===b.charAt(j-1)?0:1));',
    'diag=prev[j];prev[j]=cur;if(cur<best)best=cur;}',
    'if(best>max)return false;}',
    'return prev[b.length]<=max;}',
    'function fullText(ql){',
    'var idx=searchIndex();var out={};',
    'var words=ql.split(/[^a-z0-9_]+/).filter(function(w){',
    'return w.length>1&&STOP.indexOf(w)===-1;});',
    'if(!words.length)return out;',
    'var all=[],best=0;',
    'Object.keys(idx).forEach(function(w){',
    'var d=idx[w];var score={},cover={};',
    'words.forEach(function(word){',
    'var s=stem(word);var hit={};',
    'function mark(list,weight){',
    'if(list===undefined)return;',
    'if(typeof list==="number")list=[list];',
    'list.forEach(function(n){hit[n]=(hit[n]||0)+weight;});}',
    'mark(d.terms[s],1);mark(d.titleterms[s],5);',
    // Prefix match, so a word still being typed finds results.
    'var keys=Object.keys(d.terms),k;',
    'if(word.length>=3){',
    'for(k=0;k<keys.length;k++){',
    'if(keys[k]!==s&&keys[k].indexOf(s)===0)mark(d.terms[keys[k]],0.5);}}',
    // Only a word that matched nothing is treated as a typo, one edit at most.
    'if(word.length>=4&&!Object.keys(hit).length){',
    'for(k=0;k<keys.length;k++){',
    'if(within(s,keys[k],1))mark(d.terms[keys[k]],0.25);}}',
    // Count how many query words reached each document, alongside the score.
    'Object.keys(hit).forEach(function(n){',
    'score[n]=(score[n]||0)+hit[n];cover[n]=(cover[n]||0)+1;});});',
    'Object.keys(cover).forEach(function(n){',
    'if(cover[n]>best)best=cover[n];',
    'all.push({p:"/"+w+"/"+d.docnames[n],c:cover[n],s:score[n]});});});',
    // The best-covered tier, so one clipped word does not empty the results.
    'all.forEach(function(r){',
    'if(r.c<best)return;',
    'out[r.p]=Math.max(out[r.p]||0,r.s);});',
    'return out;}',
    // One matcher for the sidebar list and the full page, so Enter never reorders.
    'function matches(q){',
    'var ql=q.toLowerCase();var hits=[];var seen={};',
    'for(var i=0;i<D.pages.length;i++){',
    'var pg=D.pages[i];var at=pg.t.toLowerCase().indexOf(ql);',
    'var ap=at===-1?pg.p.toLowerCase().indexOf(ql):-1;',
    // One wrong letter in a half-remembered title should still find it.
    'if(at===-1&&ap===-1&&ql.length>=4){',
    'var tw=pg.t.toLowerCase().split(/[^a-z0-9]+/);',
    'for(var w=0;w<tw.length;w++){',
    'if(tw[w].length>=4&&within(ql,tw[w],1)){at=1;break;}}}',
    'if(at===-1&&ap===-1)continue;',
    // Title matches first, prefix above contains; path-only matches last.
    'hits.push({pg:pg,i:i,rank:at===0?0:(at>0?1:2)});seen[pg.p]=1;}',
    // Then whatever the body text turns up, below the title matches.
    'var ft=fullText(ql);',
    'var byPathPage={},byPathIdx={};',
    'D.pages.forEach(function(p,n){byPathPage[p.p]=p;byPathIdx[p.p]=n;});',
    'Object.keys(ft).sort(function(a,b){return ft[b]-ft[a];}).forEach(function(p){',
    'if(seen[p]||!byPathPage[p])return;',
    'hits.push({pg:byPathPage[p],i:byPathIdx[p],rank:3});});',
    'hits.sort(function(a,b){return a.rank-b.rank||(a.pg.t<b.pg.t?-1:1);});',
    'return hits;}',

    'function renderSearch(q){',
    'var hits=matches(q);',
    'var words=q.toLowerCase().split(/[^a-z0-9_]+/).filter(function(w){',
    'return w.length>1&&STOP.indexOf(w)===-1;});',
    'var shown=hits.slice(0,60);',
    'var rows=shown.map(function(h){',
    'return \'<li><a href="#\'+esc(h.pg.p)+\'"><span>\'+esc(h.pg.t)+\'</span>\'',
    '+\'<small>\'+esc(wikiName(h.pg.p))+\'</small></a>\'',
    '+\'<p class="ap-snip">\'+snippet(h.i,words)+\'</p></li>\';}).join("");',
    'doc.innerHTML="<h1>Search</h1><p>"+hits.length+" page"',
    '+(hits.length===1?"":"s")+" matching <strong>"+esc(q)+"</strong>"',
    '+(hits.length>shown.length?", showing the first "+shown.length:"")+"</p>"',
    '+(hits.length?"<ul id=\\"ap-pick\\" class=\\"ap-results\\">"+rows+"</ul>":"");',
    'crumb.textContent="";',
    'clearFooter();',
    'var sc=document.querySelector(".wy-nav-content-wrap");if(sc)sc.scrollTop=0;}',
    // A few hundred characters around the first match, from the inert block.
    'function snippet(i,words){',
    'var el=document.getElementById("p"+i);if(!el)return "";',
    'var text=el.textContent.replace(/<[^>]*>/g," ").replace(/\\s+/g," ");',
    'var low=text.toLowerCase(),at=-1,hit="";',
    'for(var w=0;w<words.length;w++){',
    'var p=low.indexOf(words[w]);',
    'if(p!==-1&&(at===-1||p<at)){at=p;hit=words[w];}}',
    'if(at===-1)return text.slice(0,160)+"\u2026";',
    'var from=Math.max(0,at-70),to=Math.min(text.length,at+hit.length+130);',
    'return (from?"\u2026":"")+esc(text.slice(from,at))+"<mark>"',
    '+esc(text.slice(at,at+hit.length))+"</mark>"+esc(text.slice(at+hit.length,to))',
    '+(to<text.length?"\u2026":"");}',

    // Typing filters the sidebar; Enter commits to the full result list.
    'var navHtml=null;',
    'function restoreNav(){if(navHtml!==null){nav.innerHTML=navHtml;navHtml=null;',
    'links=[].slice.call(nav.querySelectorAll(\'a[href^="#"]\'));',
    // The restored tree is new elements, so reopen the current branch.
    'setCurrent(current());}}',
    'function sidebarResults(q,hits){',
    'if(navHtml===null)navHtml=nav.innerHTML;',
    'var rows=hits.slice(0,40).map(function(h){',
    'return \'<li class="toctree-l1"><a href="#\'+esc(h.pg.p)+\'">\'+esc(h.pg.t)+\'</a></li>\';',
    '}).join("");',
    'nav.innerHTML=\'<p class="caption">\'+hits.length+\' result\'+(hits.length===1?"":"s")',
    '+\'</p><ul>\'+(rows||\'<li class="toctree-l1"><a href="#">nothing found</a></li>\')+\'</ul>\';}',

    'var searchTimer=null,beforeSearch=null;',
    'search.addEventListener("input",function(){',
    'clearTimeout(searchTimer);',
    'searchTimer=setTimeout(function(){',
    'var q=search.value.trim();',
    'if(q.length<2){restoreNav();',
    'if(beforeSearch!==null){var b=beforeSearch;beforeSearch=null;go(b);}return;}',
    'sidebarResults(q,matches(q));},120);});',
    'search.addEventListener("keydown",function(e){',
    'if(e.key!=="Enter")return;e.preventDefault();',
    'var q=search.value.trim();if(q.length<2)return;',
    'if(beforeSearch===null)beforeSearch=current()||"/";',
    'renderSearch(q);});',
    'window.addEventListener("hashchange",function(){beforeSearch=null;restoreNav();});',
    'document.addEventListener("keydown",function(e){',
    'if(e.key==="Escape"){var lb=document.getElementById("ap-lightbox");',
    'if(lb)lb.style.display="none";',
    'if(document.activeElement===search&&search.value){',
    'search.value="";search.dispatchEvent(new Event("input"));}}',
    'if(e.key==="/"&&document.activeElement!==search){e.preventDefault();search.focus();}});',
    'route();})();'
  ].join('');

  /* ------------------------------------------------------------ path rules */

  /** Resolve a relative href against a page path, as a browser would. */
  function resolvePath(basePath, href) {
    var parts = basePath.split('/');
    parts.pop();                       // drop the page's own filename
    href.split('/').forEach(function (seg) {
      if (seg === '..') { parts.pop(); }
      else if (seg && seg !== '.') { parts.push(seg); }
    });
    return parts.join('/');
  }

  /* -------------------------------------------------- reading the toctrees */

  /** Inner HTML of an element, counting nested tags. */
  function innerOf(html, openRe, tag) {
    var open = openRe.exec(html);
    if (!open) { return ''; }
    var from = open.index + open[0].length;
    var scan = new RegExp('<(/?)' + tag + '\\b[^>]*>', 'gi');
    scan.lastIndex = from;
    var depth = 1, m;
    while ((m = scan.exec(html)) !== null) {
      depth += m[1] ? -1 : 1;
      if (depth === 0) { return html.slice(from, m.index); }
    }
    return html.slice(from);
  }

  /** The balanced top-level <ul> blocks in a fragment, and nothing else. */
  function topLevelLists(inner) {
    var out = '', re = /<(\/?)ul\b[^>]*>/gi, depth = 0, start = -1, m;
    while ((m = re.exec(inner)) !== null) {
      if (!m[1]) {
        if (depth === 0) { start = m.index; }
        depth++;
      } else if (depth > 0) {
        depth--;
        if (depth === 0 && start >= 0) {
          out += inner.slice(start, m.index + m[0].length);
          start = -1;
        }
      }
    }
    return out;
  }

  /** Visible text of a fragment: toctree labels may carry <code> and friends. */
  function textOf(fragment) {
    return fragment.replace(/<[^>]*>/g, '').replace(/\s+/g, ' ').trim();
  }

  /** A sidebar href as a path into the file, or null for an in-page heading. */
  function navHref(raw, pagePath) {
    if (!raw) { return null; }
    if (/^(https?:|mailto:)/i.test(raw)) { return { href: raw, external: true }; }
    // The theme writes the current page as href="#".
    if (raw === '#') {
      return { href: pagePath.replace(/\.html?$/, ''), external: false };
    }
    if (raw.charAt(0) === '#') { return null; }
    var clean = raw.split('#')[0].split('?')[0];
    if (!clean) { return null; }
    // A leading slash is already a site-root path (rewrite_site_links makes them).
    var path = clean.charAt(0) === '/' ? clean : resolvePath(pagePath, clean);
    return { href: path.replace(/\.html?$/, ''), external: false };
  }

  /** Parse a sidebar fragment into nested nodes. */
  function parseToc(fragment, pagePath) {
    var root = { children: [] };
    var parents = [root];    // where the next <li> attaches
    var open = [];           // <li> elements not yet closed
    var re = /<ul\b[^>]*>|<\/ul\s*>|<li\b[^>]*>|<\/li\s*>|<a\b([^>]*)>([\s\S]*?)<\/a\s*>/gi;
    var m;
    while ((m = re.exec(fragment)) !== null) {
      var tok = m[0];
      if (/^<ul/i.test(tok)) {
        parents.push(open.length ? open[open.length - 1] : root);
      } else if (/^<\/ul/i.test(tok)) {
        if (parents.length > 1) { parents.pop(); }
        while (open.length && open[open.length - 1].depth >= parents.length) {
          open.pop();
        }
      } else if (/^<li/i.test(tok)) {
        // A missing </li> is legal, so close by depth.
        while (open.length && open[open.length - 1].depth >= parents.length) {
          open.pop();
        }
        var node = { href: null, external: false, label: '', children: [],
                     depth: parents.length };
        parents[parents.length - 1].children.push(node);
        open.push(node);
      } else if (/^<\/li/i.test(tok)) {
        open.pop();
      } else {
        var cur = open[open.length - 1];
        if (cur && cur.href === null) {
          var h = /href="([^"]*)"/i.exec(m[1] || '');
          var t = navHref(h ? h[1] : '', pagePath);
          cur.href = t ? t.href : '';
          cur.external = !!(t && t.external);
          cur.label = textOf(m[2] || '');
        }
      }
    }
    return prune(root.children);
  }

  /** Drop the entries that name no page of their own, subtree and all. */
  function prune(nodes) {
    var out = [];
    nodes.forEach(function (n) {
      if (!n.href) { return; }
      out.push({ href: n.href, external: n.external, label: n.label,
                 children: prune(n.children) });
    });
    return out;
  }

  /** The sidebar of one built page, as nodes. */
  function navNodes(html, pagePath) {
    var inner = innerOf(
      html, /<div class="wy-menu wy-menu-vertical"[^>]*>/i, 'div');
    // Only the toctree lists; the same div carries a donation form.
    var lists = topLevelLists(inner);
    return lists ? parseToc(lists, pagePath) : [];
  }

  /* ------------------------------------------------- merging the toctrees */

  /** Fold one page's sidebar into the tree; no single page carries all of it. */
  function mergeToc(into, incoming) {
    incoming.forEach(function (n) {
      var found = null;
      for (var i = 0; i < into.length; i++) {
        if (into[i].href === n.href) { found = into[i]; break; }
      }
      if (!found) {
        found = { href: n.href, external: n.external, label: n.label,
                  children: [] };
        into.push(found);
      } else if (!found.label && n.label) {
        found.label = n.label;
      }
      if (n.children.length) { mergeToc(found.children, n.children); }
    });
  }

  /** Somewhere to accumulate one merged toctree per wiki. */
  function newNav() { return { trees: {} }; }

  /** Take whatever this page's sidebar knows that the tree does not. */
  function addNav(state, html, pagePath) {
    var wiki = pagePath.split('/')[1];
    if (!wiki) { return; }
    var nodes = navNodes(html, pagePath);
    if (!nodes.length) { return; }
    if (!state.trees[wiki]) { state.trees[wiki] = []; }
    mergeToc(state.trees[wiki], nodes);
  }

  /* ------------------------------------------------ rendering the sidebar */

  /** Fallback: a flat list of the wiki's pages, when no toctree was recovered. */
  // Build-time twin of the shell's esc(): everything woven into nav HTML
  // goes through here, whatever its source.
  function escapeHtml(text) {
    return String(text).replace(/[&<>"]/g, function (c) {
      return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c];
    });
  }

  function listNav(pages, wiki) {
    var items = pages.filter(function (p) {
      return p.path.split('/')[1] === wiki;
    }).map(function (p) {
      var anchor = p.path.replace(/\.html?$/, '');
      var label = anchor.split('/').pop().replace(/[-_]/g, ' ');
      return '<li class="toctree-l1"><a class="reference internal" href="#' +
             escapeHtml(anchor) + '">' + escapeHtml(label) + '</a></li>';
    });
    return '<ul>' + items.join('') + '</ul>';
  }

  /** Flat page list for a wiki, in the same order listNav renders it. */
  function listOrder(pages, wiki) {
    return pages.filter(function (p) {
      return p.path.split('/')[1] === wiki;
    }).map(function (p) { return p.path.replace(/\.html?$/, ''); });
  }

  /** The theme's own markup, expand button included. */
  function renderNodes(nodes, level) {
    var out = '<ul>';
    nodes.forEach(function (n) {
      var kids = n.children.length ? renderNodes(n.children, level + 1) : '';
      out += '<li class="toctree-l' + level + '">' +
             '<a class="reference ' + (n.external ? 'external' : 'internal') +
             '" href="' + escapeHtml(n.external ? n.href : '#' + n.href) + '">' +
             (kids ? '<button class="toctree-expand" ' +
                     'title="Open/close menu"></button>' : '') +
             escapeHtml(n.label) + '</a>' + kids + '</li>';
    });
    return out + '</ul>';
  }

  /** The sidebar and the reading order, from one tree so they agree. */
  function buildNav(state, wikis, pages) {
    var html = '', order = [], seen = {};

    wikis.forEach(function (wiki) {
      var tree = state.trees[wiki];
      html += '<p class="caption">' + escapeHtml(wiki) + '</p>';
      if (!tree || !tree.length) {
        html += listNav(pages, wiki);
        listOrder(pages, wiki).forEach(function (p) {
          if (!seen[p]) { seen[p] = 1; order.push(p); }
        });
        return;
      }
      html += renderNodes(tree, 1);
      (function walk(nodes) {
        nodes.forEach(function (n) {
          // Cross-wiki entries belong to their own wiki's order.
          if (!n.external && n.href.charAt(0) === '/' &&
              n.href.split('/')[1] === wiki && !seen[n.href]) {
            seen[n.href] = 1;
            order.push(n.href);
          }
          walk(n.children);
        });
      })(tree);
    });

    return { html: html, order: order };
  }

  /* -------------------------------------- versioned parameter pages */

  // Parameter-list history to carry: SERIES major.minor lines, PER_SERIES each.


  // The plain /rover/docs/parameters is the latest, unversioned, always kept.
  var PARAM_PAGE = /^\/([^/]+)\/docs\/parameters-([^/]+)$/;

  /** The versioned parameter pages the file carries, labelled from filenames. */
  // Every saved version is carried: the reader chose each one at save time,
  // and silently thinning them here would lose pages they asked for.
  function parameterVersions(paths) {
    var found = {}, byWiki = {};

    paths.forEach(function (p) {
      var m = PARAM_PAGE.exec(p);
      if (!m) { return; }
      var v = /V(\d+)\.(\d+)\.(\d+)/.exec(m[2]);
      if (!v) { return; }
      if (!found[m[1]]) { found[m[1]] = []; }
      found[m[1]].push({
        p: p,
        n: m[2].split('-').join(' '),
        v: [+v[1], +v[2], +v[3]]
      });
    });

    Object.keys(found).forEach(function (w) {
      byWiki[w] = found[w].sort(function (a, b) {
        return b.v[0] - a.v[0] || b.v[1] - a.v[1] || b.v[2] - a.v[2] ||
               (a.n < b.n ? -1 : 1);
      }).map(function (e) { return { n: e.n, p: e.p }; });
    });

    return { byWiki: byWiki, drop: {} };
  }

  /* -------------------------------------------------------- the front page */

  // Picker order, most recognisable first.
  var HOME_ORDER = ['ardupilot', 'copter', 'plane', 'rover'];

  // Kept in step with DISPLAY_NAMES in scripts/build_offline_artifacts.py.
  var DISPLAY_NAMES = {
    ardupilot: 'About ArduPilot', copter: 'Copter', plane: 'Plane',
    rover: 'Rover', sub: 'Sub', blimp: 'Blimp', dev: 'Developer',
    antennatracker: 'Antenna Tracker', planner: 'Mission Planner',
    planner2: 'APM Planner 2', mavproxy: 'MAVProxy'
  };

  /** Front page and page count for each wiki in the export, in listing order. */
  function wikiHomes(index, wikis) {
    var homes = wikis.map(function (w) {
      var prefix = '/' + w + '/';
      var root = null, first = null, count = 0;
      index.forEach(function (p) {
        if (p.p.indexOf(prefix) !== 0) { return; }
        count++;
        if (!first) { first = p.p; }
        if (p.p === prefix + 'index') { root = p.p; }
      });
      return { id: w, name: DISPLAY_NAMES[w] || w, path: root || first,
               pages: count };
    }).filter(function (h) { return h.path; });

    return homes.sort(function (a, b) {
      var ai = HOME_ORDER.indexOf(a.id), bi = HOME_ORDER.indexOf(b.id);
      if (ai !== -1 || bi !== -1) {
        return (ai === -1 ? 99 : ai) - (bi === -1 ? 99 : bi);
      }
      return a.id < b.id ? -1 : 1;
    });
  }

  /* ------------------------------------------------------ the file, in parts */

  // Pages are inert <script type="text/plain"> blocks, materialised on navigation.
  function head(wikis, themeCss) {
    return '<!DOCTYPE html><html lang="en" class="writer-html5"><head>' +
      '<meta charset="utf-8">' +
      '<meta name="viewport" content="width=device-width,initial-scale=1">' +
      '<title>ArduPilot wiki (offline)</title>' +
      '<style>' + themeCss + '</style><style>' + SHELL_CSS + '</style>' +
      '</head><body class="wy-body-for-nav">' +
      '<div class="wy-grid-for-nav">' +
      '<nav data-toggle="wy-nav-shift" class="wy-nav-side">' +
      '<div id="ap-brand">ArduPilot<small>offline copy &middot; ' +
      wikis.join(', ') + '</small></div>' +
      '<input id="ap-search" placeholder="Search all pages  ( / )" autocomplete="off">' +
      '<div class="wy-menu wy-menu-vertical" id="ap-nav"></div></nav>' +
      '<section data-toggle="wy-nav-shift" class="wy-nav-content-wrap">' +
      '<div class="wy-nav-content"><div class="rst-content">' +
      '<div id="ap-bar">Offline copy built from pages saved on your device. ' +
      'It does not update itself.</div>' +
      '<div id="ap-miss"></div><div id="ap-crumb"></div>' +
      '<div itemprop="articleBody" id="ap-doc"></div>' +
      '<footer id="ap-foot"></footer>' +
      '</div></div></section></div>';
  }

  /** One page, plus any image it is the first to use. */
  function pageBlock(i, html, fresh) {
    // Any spelling ends the block: </SCRIPT>, </script >, </script/>.
    var body = html.replace(/<\/(script)/gi, '<\\/$1');
    var blocks = fresh.map(function (f) {
      return '<script type="text/plain" id="i' + f.id + '">' +
             f.uri + '<\/script>';
    }).join('');
    return blocks + '<script type="text/plain" id="p' + i + '">' +
           body + '<\/script>';
  }

  // Its own inert block, read only on the first search.
  function searchBlock(byWiki, stemmerSrc) {
    return '<script type="application/json" id="ap-fts">' +
      JSON.stringify(byWiki).split('</').join('<\\/') +
      '<\/script>' +
      (stemmerSrc
        ? '<script>' +
          // The HTML parser ends a script on "</script" in any case.
          stemmerSrc.replace(/<\/script/gi, '<\\/script') +
          '<\/script>'
        : '');
  }

  /** The routing payload and the shell that reads it. */
  function tail(payload) {
    return '<script type="application/json" id="ap-index">' +
      JSON.stringify(payload).split('</').join('<\\/') +
      '<\/script><script>' + SHELL_JS + '<\/script></body></html>';
  }

  global.ArduPilotOfflineDocument = {
    head: head,
    pageBlock: pageBlock,
    searchBlock: searchBlock,
    tail: tail,
    newNav: newNav,
    addNav: addNav,
    buildNav: buildNav,
    navNodes: navNodes,
    parameterVersions: parameterVersions,
    wikiHomes: wikiHomes,
    resolvePath: resolvePath,
    SHELL_CSS: SHELL_CSS,
    SHELL_JS: SHELL_JS
  };
})(window);
