(()=>{if("querySelector"in document&&"addEventListener"in window){var s=function(e,t){e.preventDefault(),t=t||this;var a=document.querySelectorAll(".navigation-search"),s=document.querySelectorAll(".search-item"),c=document.querySelectorAll('a[href], area[href], input:not([disabled]):not(.navigation-search), select:not([disabled]), textarea:not([disabled]), button:not([disabled]), [tabindex="0"]'),r="";t.closest(".mobile-menu-control-wrapper")&&(r=document.getElementById("site-navigation"));for(var o=0;o<a.length;o++)if(a[o].classList.contains("nav-search-active")){if(!a[o].closest("#sticky-placeholder")){a[o].classList.remove("nav-search-active");var i=document.querySelector(".has-active-search");i&&i.classList.remove("has-active-search");for(var l=0;l<s.length;l++){s[l].classList.remove("close-search"),s[l].classList.remove("active"),s[l].querySelector("a").setAttribute("aria-label",generatepressNavSearch.open);for(var n=0;n<c.length;n++)c[n].closest(".navigation-search")||c[n].closest(".search-item")||c[n].removeAttribute("tabindex")}document.activeElement.blur()}}else if(!a[o].closest("#sticky-placeholder")){var i=a[o].closest(".toggled"),d=(i&&i.querySelector("button.menu-toggle").click(),r&&r.classList.add("has-active-search"),a[o].classList.add("nav-search-active"),this.closest("nav"));for(d&&(d=(d=d.classList.contains("mobile-menu-control-wrapper")?r:d).querySelector(".search-field"))&&d.focus(),l=0;l<s.length;l++){for(s[l].classList.add("active"),s[l].querySelector("a").setAttribute("aria-label",generatepressNavSearch.close),n=0;n<c.length;n++)c[n].closest(".navigation-search")||c[n].closest(".search-item")||c[n].setAttribute("tabindex","-1");s[l].classList.add("close-search")}}};if(document.body.classList.contains("nav-search-enabled")){for(var e=document.querySelectorAll(".search-item"),t=0;t<e.length;t++)e[t].addEventListener("click",s,!1);document.addEventListener("keydown",function(e){if(document.querySelector(".navigation-search.nav-search-active")&&"Escape"===e.key)for(var t=document.querySelectorAll(".search-item.active"),a=0;a<t.length;a++){s(e,t[a]);break}},!1)}}})();
;