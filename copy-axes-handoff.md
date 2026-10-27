# Copy axes handoff: us-english and slide-density

How the kickoff deck's copy was checked, for reuse in the orchestrator's axes-copy loop. The writer is Claude; the checker is GPT via `codex exec` with read access to this repository; the writer refines only from quoted findings; a fresh subagent does a final cold read.

## Checker axis text (as sent to the checker)

Both are appended to the six axes already in use (faithful, rubric, clear, concise, voice, no-ai-tells) and the speaks-to-audience axis. One verdict per axis; a failure must quote the exact words and say why.

```
- us-english: American spelling and usage throughout (meter, color, practice as a verb, memorize, neighbor, center, modeled, gray). Quote any British spelling or usage and give the American form.

- slide-density: a slide shows as few words as the point needs; the explanation belongs in the spoken notes. Fail a slide whose on-screen text (everything except the spoken notes) is over about 40 words, or over 90 for a glossary or table slide, and fail any on-screen sentence that would be clearer spoken than read. Quote the words to cut or move to the notes. Each slide's on-screen word count is given in the text as [N words on screen]. Does not apply to the sim page or README.
```

Each slide's text is handed to the checker with its count, e.g. `--- slide sensor [38 words on screen] ---`, and speaker notes are marked `[speaker notes]` so the checker can tell the two apart.

## The same rules as repository review rules (`.review/corpus.json`, kind `copy`)

```
COPY-TEACH-US-ENGLISH: Human-facing text (deck, sim page, README, Driver Station strings) uses American spelling and usage: meter, color, practice (verb), memorize, neighbor, center, modeled, gray. A finding wherever a British form appears.

COPY-TEACH-SLIDE-DENSITY: A slide shows as few words as its point needs and the explanation lives in the speaker notes. A finding for any slide whose on-screen text exceeds about 40 words (90 for a glossary or table slide), or whose on-screen text is a paragraph that would be clearer spoken.
```

Spellings actually found and fixed in this repo: practise, metre(s), centimetres, neighbouring, memorise, modelled, centre.

## Three slides, before and after

Before = on-screen text as it stood when the slide-density rule arrived. After = on-screen text now, with the speaker notes that carry what came off the screen.

### Cover (`docs/kickoff-deck/cover.html`)

**Before, on screen (84 words):**

```
FTC 18121 · software kickoff
Shooter Brain
Our robot can see the goal. Tonight you learn how it works out how far away the goal is, how fast to spin the wheel, and when to shoot. Then you build that part yourself.
Tonight you practise the shooter's decisions in a browser, change its code, check it with automated tests (short checks that the code still does its job), and take on the team challenge.
The practice page is a file in our code: TeamCode/sim/shooter-brain-testbed.html
```

**After, on screen (24 words):**

```
FTC 18121 · software kickoff
Shooter Brain
How the robot decides how far, how fast, and when. Then you build it.
Practice page: TeamCode/sim/shooter-brain-testbed.html
```

**Speaker notes now (66 words):**

> Our robot can see the goal. The part that decides what to do with that is small enough that you can build it yourselves, in a practice page in your browser, and later it goes on the real robot. By the end of tonight you can open that page, build a shooter brain, prove it works with tests, and know how it gets onto the robot.

### One pipeline, three kinds of stage (`docs/kickoff-deck/pipeline.html`)

**Before, on screen (142 words):**

```
One pipeline, three kinds of stage
input
tag ranges
RANGE FILTERS
How far? A better distance
TagMedian · OutlierReject · RollingAverage
lookup
velocity table
VELOCITY MODIFIERS
How fast? A better RPM
FarShotBoost · RangeTrim · ManualOffset
FIRE GATES
When? Fire, or a reason to hold
RecentTagGate · VelocityToleranceGate · TagCountGate
Every stage has a name and an on/off switch, and most have tunables: numbers you can adjust. Range filters run in their listed order, then velocity modifiers; each fire gate checks the same shot information. With no stages at all you get the baseline: first tag, table value, fire every cycle.
OutlierReject(maxDevM=0.3); TagMedian; RollingAverage(frames=5); !FarShotBoost(startM=2.5, pctPerM=2); RecentTagGate(maxAgeMs=300); VelocityToleranceGate(tolRpm=60)
That one line is the spec for the whole pipeline. Semicolons separate stages, parentheses hold the numbers, a leading ! means switched off. The simulator writes this line; ShooterConfig.java on the robot reads it.
```

**After, on screen (28 words):**

```
One pipeline, three kinds of stage
tag ranges
RANGE FILTERS
How far?
velocity table
VELOCITY MODIFIERS
How fast?
FIRE GATES
When?
OutlierReject(maxDevM=0.3); TagMedian; RollingAverage(frames=5); !FarShotBoost(startM=2.5, pctPerM=2); RecentTagGate(maxAgeMs=300); VelocityToleranceGate(tolRpm=60)
```

**Speaker notes now (129 words):**

> Every stage has a name and an on/off switch, and most have tunables, numbers you can adjust. Range filters run in the order you list them, then the velocity modifiers; each fire gate checks the same shot information and can hold the shot. With no stages at all you get the baseline: first tag, table value, fire every cycle. That line of text is the spec. Semicolons separate stages, the numbers in parentheses are the tunables, and a leading exclamation mark means switched off. Read it out loud, stage by stage, and you have described a whole brain in one breath. The simulator writes this line for you, and ShooterConfig.java on the robot reads it, so you can share a pipeline with anyone on the team as plain text.

### Team challenge (`docs/kickoff-deck/challenge.html`)

**Before, on screen (137 words):**

```
Team challenge
Build a brain that beats the mentor's
Start
Press Empty, then Wander, noise 1×, drag 1×. Press Reset stats.
Build
Add one stage at a time. Run 60 seconds. Write down what changed and why before the next one.
Win
Match or beat the mentor reference's hit rate before you press Reveal in the stats panel. Then press it and compare.
Report
Your spec line. Hit rate and shots per minute from a 60-second run. One sentence per stage: what it reads, what it changes, what it cost you.
Make your own branch from feature/shooter-vision-testbed: a separate copy of the code. When the tests pass, open a pull request back to it, a request for a mentor to check and combine your changes. Calibration night is when we run your pipeline on the real robot.
```

**After, on screen (50 words):**

```
Team challenge
Beat the mentor's brain
Start
Empty pipeline. Wander. Reset stats.
Build
One stage at a time. 60 seconds each.
Win
Match or beat the reference. Then press Reveal.
Report
Spec. Hit rate. Shots per minute. One line per stage.
Your own branch. Tests pass. Pull request. Calibration night.
```

**Speaker notes now (188 words):**

> Start with the Empty pipeline preset, set Wander, leave noise and drag at one, and press Reset stats. Add one stage at a time, run 60 seconds, and write down what changed and why before you add the next. Your goal is to match or beat the mentor reference's hit rate before you press the Reveal button in the stats panel; then press it and compare. Report your spec line, your hit rate and shots per minute from a 60-second run, and one sentence per stage: what it reads, what it changes, and what it cost you, such as more time held. Work on your own branch, which is a separate copy of the code made from feature/shooter-vision-testbed. When the tests pass, open a pull request back to it; that is a request for a mentor to check your changes and combine them. If you beat the reference, bring your spec to that review. You may also write a stage nobody has built yet; if you do, send its Java and its test with the report. Calibration night is when we run your pipeline on the real robot.

## Word counts after the pass

| Slide | Words on screen |
|---|---|
| cover | 24 |
| primer-shooter | 35 |
| primer-words | 89 |
| primer-table | 49 |
| problem | 35 |
| sensor | 38 |
| pipeline | 28 |
| sim | 39 |
| code | 39 |
| workflow | 39 |
| challenge | 50 |

The glossary slide (primer-words) sits at 89 under the 90-word allowance for a table slide.
