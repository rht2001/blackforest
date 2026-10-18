/* Local CSV decoding only. No eval, hardware reads, or invented XYZ columns. */
(function(root) {
  'use strict';
  function records(text) {
    text=text.replace(/^\uFEFF/,'');
    const first=text.split(/\r?\n/,1)[0],delimiter=first.includes(';')?';':',';
    const rows=[];let row=[],field='',quoted=false;
    for(let i=0;i<text.length;i++){
      const c=text[i];
      if(c==='"') {if(quoted&&text[i+1]==='"'){field+='"';i++}else quoted=!quoted}
      else if(c===delimiter&&!quoted){row.push(field);field=''}
      else if((c==='\n'||c==='\r')&&!quoted){if(c==='\r'&&text[i+1]==='\n')i++;row.push(field);if(row.some(x=>x.trim()))rows.push(row);row=[];field=''}
      else field+=c;
    }
    if(quoted)throw Error('CSV contains an unfinished quoted field.');
    row.push(field);if(row.some(x=>x.trim()))rows.push(row);
    if(rows.length<2)throw Error('CSV needs a header and at least one data row.');
    const header=rows.shift().map(x=>x.trim());
    if(new Set(header).size!==header.length)throw Error('CSV contains duplicate column names.');
    if(!header.includes('angles'))throw Error('CSV needs an angles column with six actual readings.');
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
    const angles=numericArray(row.angles);
    if(!Array.isArray(angles)||angles.length!==6||!angles.every(x=>finite(x)&&x>=0&&x<=1000))throw Error('This row does not contain six valid angles in 0..1000.');
    const matrix=numericArray(row.tactile_matrix),tactile_regions={};
    if(matrix!==null){
      if(matrix.length!==112||matrix.some(r=>!Array.isArray(r)||r.length!==17))throw Error('tactile_matrix must be a complete 112 × 17 array; truncated exports cannot be reconstructed.');
      regions.forEach((region,column)=>{
        const values=matrix.slice(0,region.rows*region.cols).map(r=>r[column]);
        if(values.every(x=>finite(x)&&x>=0&&x<=4095))tactile_regions[region.key]=Array.from({length:region.rows},(_,r)=>values.slice(r*region.cols,(r+1)*region.cols));
      });
    }
    return {angles_act:angles,tactile_regions,index:row.index||row._row,state:row.state||''};
  }
  root.IdentificationHistory={records,numericArray,telemetry};
  if(typeof module!=='undefined')module.exports=root.IdentificationHistory;
})(typeof globalThis==='undefined'?this:globalThis);
