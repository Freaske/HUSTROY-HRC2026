#pragma once

constexpr char TEST_PAGE[] = R"HTML(<!doctype html>
<html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>HUSTROY · Test xe</title><style>
*{box-sizing:border-box}body{font:1rem system-ui,sans-serif;background:#eff4f1;color:#17352a;margin:0}
main{max-width:38rem;margin:auto;padding:1.25rem}h1{font-size:1.55rem;margin:.5rem 0 1.25rem}
section,form{padding:1rem;background:#fff;border:1px solid #c7d8ce;border-radius:.65rem;margin:0 0 1rem}
label,strong{display:block;font-weight:700;margin-bottom:.5rem}input,button{font:inherit;padding:.6rem;border-radius:.35rem}
input{width:100%;border:1px solid #729185;margin-bottom:.5rem}button{border:1px solid #356552;background:#e6f4ee;color:#17352a}
button.stop{background:#a32025;color:#fff;border-color:#a32025}.buttons{display:flex;flex-wrap:wrap;gap:.5rem}
p,pre{margin:.5rem 0 0}pre{white-space:pre-wrap;overflow-wrap:anywhere}
</style></head><body><main>
<h1>HUSTROY · Test xe</h1>
<section aria-label="Điều khiển"><div class="buttons">
<button data-cmd="MODE LINE_FOLLOW">Bám line</button><button data-cmd="ARM">ARM</button>
<button data-cmd="GO">GO</button><button class="stop" data-cmd="STOP">STOP</button>
<button data-cmd="DISARM">DISARM</button></div></section>
<form><label for="command">Lệnh</label><input id="command" maxlength="63" autocapitalize="characters" autocomplete="off" placeholder="KP=1.2, KD=6 hoặc SPD=120">
<div class="buttons"><button type="submit">Gửi</button><button type="button" data-cmd="GET">GET</button>
<button type="button" data-cmd="SAVE">SAVE</button></div>
<p id="reply" role="status"></p></form>
<section><strong>Log trực tiếp</strong><pre id="log">Đang kết nối…</pre></section>
<script>
const command=document.querySelector('#command'), reply=document.querySelector('#reply'), log=document.querySelector('#log');
async function send(value){
  if(!value)return;
  try{
    const response=await fetch('/command',{method:'POST',headers:{'X-Hustroy-Control':'1'},body:value});
    reply.textContent=value+' → '+await response.text();
  }catch{reply.textContent='Mất kết nối với xe';}
}
document.querySelector('form').onsubmit=event=>{event.preventDefault();send(command.value.trim().toUpperCase());};
for(const button of document.querySelectorAll('[data-cmd]'))button.onclick=()=>send(button.dataset.cmd);
async function refresh(){
  const timeout=new AbortController();
  const deadline=setTimeout(()=>timeout.abort(),1000);
  try{
    const response=await fetch('/log',{cache:'no-store',signal:timeout.signal});
    if(!response.ok)throw Error();
    log.textContent=await response.text();
  }catch{log.textContent='Mất kết nối với xe';}
  finally{clearTimeout(deadline);setTimeout(refresh,100);}
}
refresh();
</script></main></body></html>)HTML";
