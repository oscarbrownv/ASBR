# Project Overview

Double-click `ASBR.prj` first when you open the project in MATLAB.

## Publishing the Coursework to a New GitHub Repository

Follow the checklist below to avoid accidentally staging files from outside this project.

1. **Open a terminal inside the project folder.**
   ```bash
   cd /workspace/ASBR   # or wherever you cloned/extracted the coursework
   pwd                  # confirm the prompt ends with /ASBR
   ls                   # you should see THA1, THA2, THA3, THA4, etc.
   ```
   If you do not see this structure, move into the project directory before running any Git commands.

2. **Ensure you are working with the intended Git repository.**
   ```bash
   git status
   ```
   * If Git reports `fatal: not a git repository`, initialize the repo with `git init` while still inside `/ASBR`.
   * If the status output lists thousands of files you do not recognize, you are probably in the wrong directory—press `Ctrl+C`, change into `/ASBR`, and run `git status` again.

3. **Connect the local folder to your empty GitHub repository.**
   ```bash
   git remote add origin https://github.com/<your-user>/<new-repo>.git
   git branch -M main  # optional: align the branch name with GitHub
   ```
   Run `git remote -v` to double-check the URL before you push.

4. **Stage only the coursework files.**
   *Preferred:* `git add -A` or `git add .` while inside `/ASBR`. Either command stages everything beneath the current directory, so being in the project root is critical.
   *Safety tip:* Run `git status --short` right after staging. The output should show paths like `THA1/…`, `THA2/…`, etc. If anything outside the project appears, use `git reset` to unstage, move into `/ASBR`, and retry.

5. **Create a descriptive commit.**
   ```bash
   git commit -m "Initial import of coursework materials"
   ```

6. **Push to GitHub and set the upstream branch.**
   ```bash
   git push -u origin main
   ```

7. **Verify the upload.**
   Refresh the repository page on GitHub. You should see the commit, the MATLAB project file, and the THA folders. If not, confirm you pushed to the correct remote URL and branch.

These steps keep Git focused on the coursework directory so you do not accidentally stage unrelated files from your computer.
