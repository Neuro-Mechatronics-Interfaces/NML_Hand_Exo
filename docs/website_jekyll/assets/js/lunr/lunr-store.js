---
layout: null
---
var store = [
  {%- for c in site.pages -%}
    {%- unless c.url contains 'assets' or c.url contains '404' -%}
      {%- if c.search != false -%}
        {
          "title": {{ c.title | jsonify }},
          "excerpt": {{ c.content | strip_html | truncatewords: 50 | jsonify }},
          "content": {{ c.content | strip_html | jsonify }},
          "url": {{ c.url | relative_url | jsonify }}
        }{%- unless forloop.last -%},{%- endunless -%}
      {%- endif -%}
    {%- endunless -%}
  {%- endfor -%}
]
