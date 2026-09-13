const { chromium } = require("playwright");
const { pathToFileURL } = require("url");
const path = require("path");

const root = path.resolve(__dirname, "..");
const replay = path.join(root, "res", "replays", "问题三_实际演练回放_CW3X.html");
const outputDir = path.join(root, "res", "replays");

(async () => {
  const browser = await chromium.launch({ headless: true });
  const page = await browser.newPage({ viewport: { width: 1400, height: 1000 } });
  const errors = [];
  page.on("pageerror", error => errors.push(error.message));
  await page.goto(pathToFileURL(replay).href);
  await page.locator("#end").click();
  await page.screenshot({
    path: path.join(outputDir, "replay-desktop.png"),
    fullPage: true,
  });
  console.log("END", await page.locator("#live").innerText());
  console.log("clear channels", await page.locator(".channels .cleared").count());
  await page.locator("#ch1").click();
  console.log("filter", await page.locator("#filterlabel").innerText());
  await page.locator("#reset").click();
  console.log("START", await page.locator("#live").innerText());
  await page.locator("#play").click();
  await page.waitForTimeout(400);
  await page.locator("#play").click();
  console.log("PLAY", await page.locator("#clock").innerText());
  await page.setViewportSize({ width: 390, height: 844 });
  await page.screenshot({
    path: path.join(outputDir, "replay-mobile.png"),
    fullPage: true,
  });
  console.log(
    "overflow",
    await page.evaluate(() => document.documentElement.scrollWidth > innerWidth),
  );
  console.log("errors", errors);
  if (errors.length) process.exitCode = 1;
  await browser.close();
})().catch(error => {
  console.error(error);
  process.exit(1);
});
