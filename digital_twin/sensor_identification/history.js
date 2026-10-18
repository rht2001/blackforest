/* Local CSV decoding only. No eval, hardware reads, or invented XYZ columns. */
(function(root) {
  'use strict';
  const angleColumns=['little','ring','middle','index','thumb_bend','thumb_rotate'].map(k=>'angle_'+k+'_raw');
  function records(text) {
    text=text.replace(/^\uFEFF/,'');
    const first=text.split(/\r?\n/,1)[0],delimiter=first.includes(';')?';':',';
    const rows=[];let row=[],pieces=[],start=0,quoted=false;
    // Slice complete fields instead of allocating one string node per character
    // in the large pressure matrices (combined run files exceed 100 MB).
    function field(end){const value=pieces.length?pieces.join('')+text.slice(start,end):text.slice(start,end);pieces=[];return value}
    for(let i=0;i<text.length;i++){
      const c=text[i];
      if(c==='"') {
        pieces.push(text.slice(start,i));
        if(quoted&&text[i+1]==='"'){pieces.push('"');i++}else quoted=!quoted;
        start=i+1;
      }
      else if(c===delimiter&&!quoted){row.push(field(i));start=i+1}
      else if((c==='\n'||c==='\r')&&!quoted){row.push(field(i));if(c==='\r'&&text[i+1]==='\n')i++;if(row.some(x=>x.trim()))rows.push(row);row=[];start=i+1}
    }
    if(quoted)throw Error('CSV contains an unfinished quoted field.');
    row.push(field(text.length));if(row.some(x=>x.trim()))rows.push(row);
    if(rows.length<2)throw Error('CSV needs a header and at least one data row.');
    const header=rows.shift().map(x=>x.trim());
    if(new Set(header).size!==header.length)throw Error('CSV contains duplicate column names.');
    if(!header.includes('angles')&&!angleColumns.every(k=>header.includes(k)))throw Error('CSV needs an angles column or all six angle_*_raw columns.');
    if(rows.some(r=>r.length!==header.length))throw Error('CSV row width does not match its header.');
    return rows.map((values,i)=>Object.fromEntries(header.map((key,j)=>[key,values[j]??'']).concat([['_row',i+1]])));
  }
  function numericArray(text) {
    if(typeof text!=='string'||!text.trim())return null;
    try {const value=JSON.parse(text);if(Array.isArray(value))return value}catch{}
    // Older NumPy exports use spaces instead of commas. Parse that grammar
    // explicitly; truncated arrays ("...") are rejected rather than guessed.
    const tokens=[];const pattern=/\[|\]|,|[-+]?(?:\d+\.?\d*|\.\d+)(?:e[-+]?\d+)?|nan|none|null/gi;
    let last=0,match;
    while((match=pattern.exec(text))){if(text.slice(last,match.index).trim())throw Error('Unsupported or truncated numeric array.');tokens.push(match[0]);last=pattern.lastIndex}
    if(text.slice(last).trim())throw Error('Unsupported or truncated numeric array.');
    let i=0;
    function value(){const token=tokens[i++];if(token==='['){const out=[];while(i<tokens.length&&tokens[i]!==']'){if(tokens[i]===','){i++;continue}out.push(value())}if(tokens[i++]!==']')throw Error('Unfinished numeric array.');return out}if(token===undefined||token===']'||token===',')throw Error('Invalid numeric array.');return /^(nan|none|null)$/i.test(token)?null:Number(token)}
    const out=value();if(i!==tokens.length||!Array.isArray(out))throw Error('Expected a numeric array.');return out;
  }
  const finite=x=>typeof x==='number'&&Number.isFinite(x);
  function telemetry(row,regions) {
    const angles=row.angles?.trim()?numericArray(row.angles):angleColumns.map(k=>String(row[k]??'').trim()?Number(row[k]):null);
    if(!Array.isArray(angles)||angles.length!==6||!angles.every(x=>finite(x)&&x>=0&&x<=1000))throw Error('This row does not contain six valid angles in 0..1000.');
    const matrix=numericArray(row.tactile_matrix),tactile_regions={};
    if(matrix!==null){
      if(matrix.length!==112||matrix.some(r=>!Array.isArray(r)||r.length!==17))throw Error('tactile_matrix must be a complete 112 × 17 array; truncated exports cannot be reconstructed.');
      regions.forEach((region,column)=>{
        const values=matrix.slice(0,region.rows*region.cols).map(r=>r[column]);
        if(values.every(x=>finite(x)&&x>=0&&x<=4095))tactile_regions[region.key]=Array.from({length:region.rows},(_,r)=>values.slice(r*region.cols,(r+1)*region.cols));
      });
    }else{
      regions.forEach(region=>{
        const keys=Array.from({length:region.rows*region.cols},(_,i)=>region.key+'_'+String(i).padStart(3,'0'));
        if(!keys.every(k=>Object.hasOwn(row,k)))return;
        const values=keys.map(k=>String(row[k]??'').trim()?Number(row[k]):null);
        if(values.every(x=>finite(x)&&x>=0&&x<=4095))tactile_regions[region.key]=Array.from({length:region.rows},(_,r)=>values.slice(r*region.cols,(r+1)*region.cols));
      });
    }
    return {angles_act:angles,tactile_regions,index:row.index||row._row,state:row.state||''};
  }
  root.IdentificationHistory={records,numericArray,telemetry};
  if(typeof module!=='undefined')module.exports=root.IdentificationHistory;
})(typeof globalThis==='undefined'?this:globalThis);
