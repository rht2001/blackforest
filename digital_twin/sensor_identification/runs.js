/* Compatibility entry point; the canonical builder ships with history.js. */
(function(root){
 'use strict';
 if(typeof module!=='undefined')module.exports=require('./history.js').runs;
 else root.IdentificationRuns=root.IdentificationHistory.runs;
})(typeof globalThis==='undefined'?this:globalThis);
