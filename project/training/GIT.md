# Git for the Training Lessons

The team uses **git** to share code and **GitHub** to review it. In these lessons, you'll save your work the same way
the team does: on your own branches, pushed to the team's repository, reviewed through pull requests.

Everything here uses **VS Code**. Terminal commands are listed too, because you'll see them in other guides, and
because the terminal is sometimes faster.

## Your branches

You get your own set of branches. Nothing you do on them can affect the real robot code.

```
main                          ← the team's real code. You never commit or push here
└── training/<you>/main       ← YOUR main: your finished lessons collect here
    ├── training/<you>/lesson-1
    ├── training/<you>/lesson-2
    └── ...                   ← one branch per lesson, merged into YOUR main when done
```

Use your GitHub username (lowercase) for `<you>`, for example `training/jsmith/lesson-3`.

> [!IMPORTANT]
> Your lesson branches merge into **`training/<you>/main`**, never into `main`. GitHub suggests `main` by default
> when you open a pull request, so always check the **base** branch before you click Create.

## One-time setup (Lesson 0)

1. **GitHub access.** Make a GitHub account if you don't have one, and send your username to a mentor. They'll add you to
   the team so you can push branches. Accept the invitation email.
2. **Tell git who you are.** In a terminal:
   ```bash
   git config --global user.name "Your Name"
   git config --global user.email "you@example.com"   # the email on your GitHub account
   git config --global pull.rebase false               # "git pull" merges (the lessons assume this)
   ```
3. **Open the workspace in VS Code**: `code ~/code/lance-ws`.
4. **Find the right repository.** Open the **Source Control** view (`Ctrl` + `Shift` + `G`). The workspace contains several
   git repositories (the robot code, plus a few libraries it includes). Use the one named **`src`**. If you only see one
   repository at a time, click **…** at the top of the view, then **Views** → **Repositories**, to choose between them.
5. **Install the GitHub Pull Requests extension**: open Extensions (`Ctrl` + `Shift` + `X`), search for
   "GitHub Pull Requests", and install the one published by **GitHub**.
6. **Sign in.** The first time you push, VS Code asks to sign in with GitHub. Click **Allow**, and finish in your browser.
   This only works from VS Code (the Source Control view or VS Code's own terminal), so push from there.

## Everyday cheat sheet

| I want to… | In VS Code | In a terminal (from `~/code/lance-ws/src`) |
|------------|------------|--------------------------------------------|
| See what I changed | Source Control view. Click a file to see its **diff** (before and after, side by side) | `git status`, `git diff` |
| See which branch I'm on | Bottom-left corner of the window (the status bar) | `git branch --show-current` |
| Start a new branch | Click the branch name (bottom left) → **Create new branch from…** → pick the starting branch | `git switch -c <new> <start>` |
| Switch branches | Click the branch name (bottom left) → pick a branch | `git switch <branch>` |
| Choose what to commit (**stage**) | Hover over a file → **+** | `git add <file>` |
| Commit | Type a message in the box at the top of Source Control → **Commit** | `git commit -m "message"` |
| Push my commits to GitHub | **Publish Branch** (first time) or **Sync Changes** | `git push -u origin HEAD` (first time), then `git push` |
| Get the latest from GitHub | **Sync Changes**, or **…** → **Pull** | `git pull` |
| Undo my changes to a file | Hover over the file → **Discard Changes** (↶). **This can't be undone** | `git restore <file>` |
| Put changes aside for later | **…** → **Stash** → **Stash** | `git stash`, then `git stash pop` |
| Fix my last commit, **if I haven't pushed it yet** | Arrow next to **Commit** → **Commit (Amend)** | `git commit --amend` |
| Undo my last commit (keep the changes) | **…** → **Commit** → **Undo Last Commit** | `git reset --soft HEAD~1` |
| Merge another branch into this one | `Ctrl` + `Shift` + `P` → type **Git: Merge** → pick the branch | `git merge <branch>` |
| See the history | The **Graph** section at the bottom of Source Control. For one file: **Timeline** at the bottom of the Explorer | `git log --oneline --graph` |

> [!WARNING]
> If nothing is staged and you click **Commit**, VS Code offers to commit **everything**. Say no, unless you've checked
> every file in the list. Commit only what belongs together.

### Good commits
- **Commit at every ✅ checkpoint.** Small commits are easy to review and easy to undo.
- **Write messages that say what and why**: `Add high-current warning to motor monitor`, not `stuff` or `fix`. Start with
  a verb, and keep the first line under about 60 characters.
- **Never commit** `build/`, `install/`, or `log/` (they're outside `src/`, so git doesn't see them anyway), passwords
  or tokens, or large files like bags and videos.

## The lesson routine

From Lesson 1 on, every lesson follows the same pattern:

1. **Start a branch** for the lesson from **your** main: click the branch name (bottom left) →
   **Create new branch from…** → `training/<you>/main` → name it `training/<you>/lesson-N`.
2. **Work and commit** at each checkpoint.
3. **Push**: **Publish Branch**, then **Sync Changes** after later commits.
4. **Finish**: Lessons 1–2 merge into your main yourself. From Lesson 3 on, open a pull request (below).
5. **Switch back** to `training/<you>/main` and **Sync Changes**, so it has the merged lesson before you start the next one.

## Pull requests

A **pull request** (PR) asks for one branch to be merged into another, and gives a place to review it first. This is how
every change gets into the robot code, so it's worth getting comfortable with.

**Open one in VS Code:** in the Source Control view, click the **Create Pull Request** icon at the top of the view
(or `Ctrl` + `Shift` + `P` → **GitHub Pull Requests: Create Pull Request**). Then:
- Set **base** to `training/<you>/main`. ⚠️ Not `main`!
- Set **merge** (the branch being merged) to your lesson branch.
- Give it a title, describe what you did, and **Create**.

**Or on github.com:** after pushing, GitHub shows a **Compare & pull request** banner on the repository page. Change the
**base** dropdown before you create it.

**Ask your mentor to review it.** In the PR, pick them under **Reviewers**. Their comments show up on GitHub and, with the
extension, right next to your code in VS Code. To respond to a comment, fix the code, commit, and push. The PR updates by
itself. Reply to the comment to say what you changed, or why you didn't. When your mentor approves, click
**Merge pull request** on GitHub.

> [!WARNING]
> Don't **amend** a commit you've already pushed. Amending replaces the commit with a new one, so GitHub's copy and
> yours no longer match, and the push is rejected. Once a commit is pushed, fix things with a *new* commit.

## Merge conflicts

When two branches change the **same lines**, git can't decide which version to keep. You decide, then finish the merge.
Lesson 5 makes this happen on purpose. In VS Code:

1. Files with conflicts are listed under **Merge Changes** in Source Control. Open one.
2. Each conflict shows both versions, with buttons above them: **Accept Current Change** (yours), **Accept Incoming
   Change** (theirs), and **Accept Both Changes**. For a larger view, click **Resolve in Merge Editor**.
3. Often the right answer is *both, combined by hand*. Edit the result until it's correct. Then **check it still builds
   and runs**: a resolved conflict can still be wrong.
4. Stage the file (**+**), or click **Complete Merge** in the merge editor, then **Commit**.

To give up and go back to before the merge: `Ctrl` + `Shift` + `P` → **Git: Abort Merge** (`git merge --abort`).

## Keeping up with the team

`main` keeps moving while you work. To bring the team's changes into your branches:

1. Switch to `training/<you>/main`.
2. `Ctrl` + `Shift` + `P` → **Git: Fetch** (this downloads new commits without changing your files).
3. **Git: Merge** → pick `origin/main`. Fix any conflicts, then **Sync Changes**.

Then rebuild: someone may have changed code you depend on.

## Oops

| Problem | Fix |
|---------|-----|
| I committed to the wrong branch | **Undo Last Commit** (the changes come back), switch branches, and commit again |
| I committed on `main` by accident | Don't push. **Undo Last Commit**, create your branch, commit there. If you already pushed, tell a mentor |
| VS Code won't switch branches because of my changes | Commit them, or stash them first |
| Push was rejected ("fetch first" / "non-fast-forward") | Someone, maybe you on another computer, pushed to this branch. **Sync Changes** merges their work with yours |
| "detached HEAD" | You checked out a commit, not a branch. Create a branch right there if you made changes, otherwise switch to a branch |
| I'm lost | `git status` explains where you are and usually suggests what to do next. Then ask a mentor. Nothing is lost once it's committed |

## Finishing up

After Lesson 7, delete your merged lesson branches, so the repository's branch list stays tidy: on GitHub, use the
**Delete branch** button after each PR is merged. Locally, `Ctrl` + `Shift` + `P` → **Git: Delete Branch…**.
Keep `training/<you>/main`: it's your portfolio of finished lessons.

---

Want to go deeper? The free book [Pro Git](https://git-scm.com/book/en/v2) is the standard reference. Chapters 2 and 3
cover everything here.
