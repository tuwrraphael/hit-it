const fs = require("fs");
let text = fs.readFileSync("./outfile3", "utf-8");
text = text.replace(/\|(.+)/g, "");
text = text.replace(/\[.+/g, "");
text = text.replace(/button|start recording|start logging/g, "");
text = text.replace(/\s+/g, " ");
var numbers = text.split(" ").filter(x => x != "")
    // .map(t => parseInt(t, 16));
let arr = new Uint8Array(numbers.length);
let view = new DataView(arr.buffer);
for (let i = 0; i < numbers.length; i++) {
    view.setUint8(i,parseInt(numbers[i],16));
}
let mapped = []
for(let i = 0; i < numbers.length; i+=2) {
    mapped.push(view.getInt16(i,true));
}
console.log("TS,NUM");
console.log(
    mapped.map((t, i) => `${i * 0.125},${t}`)
        .join("\n")
);