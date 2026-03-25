# Commits and Pull Requests

Effective use of Git for version control, including clear commit messages and well-managed pull requests, is fundamental to collaborative software development.

## Pulling Main (Keeping Your Branch Up-to-Date)

Regularly pulling changes from the `main` branch into your feature branch is crucial.

*   **Why?**
    *   **Stays Up-to-Date**: Keeps your branch synchronized with the latest changes in the main codebase.
    *   **Avoids Merge Conflicts**: Minimizes the chances of significant merge conflicts, making your pull request process smoother.
    *   **Easier Pull Requests**: Makes integrating your changes back into `main` as easy as possible.
*   **How?**
    *   **GitHub Desktop**: In GitHub Desktop, you can typically use the "Update from Main" option under the "Branch" menu.
    *   **`git` CLI**: `git pull origin main` (or `git pull --rebase origin main` for a cleaner history).
    *   **Beware**: Changes may not always be seamless. If conflicts arise, you'll need to resolve them. Git provides options to "keep incoming changes" or "keep current changes" for individual conflicting files.

## Pull Requests (PRs)

Pull Requests are how we propose changes to the codebase and get them reviewed before merging into `main`.

### Streamlining Code

*   **Compatibility**: Before creating a PR, ensure your branch is compatible with `main`. Regularly pulling `main` into your branch helps achieve this. Do this with `git pull origin` and `git merge origin/main`.
*   **Ask for a Code Reviewer**: A programming lead or a person you collaborated with on the feature can review.
*   **Ensure Your Branch Builds**: This is critical! If your branch does not build, it poses a major risk when merging into `main`. A non-building branch will break the robot code for everyone. Build with `./gradlew clean build`. *NEVER* merge into `main` if the ci fails!
    *   **Build Breaks => Robot Breaks**

## Commits

Commits are snapshots of your changes in the repository. They should be understandable and useful.

*   **Purpose**: Commits should explain *why* a change was made, not just *what* was changed. This is especially helpful for future debugging or understanding code evolution.
*   **Audience**: If one person is using the branch, the commits need to be understandable for them. If many are using the same branch, they need to be accessible for all.

### Writing Summaries

*   **Concise**: Keep commit messages concise but informative. The first line should be a short summary of the change.
*   **Detailed (if needed)**: Follow with a more detailed body if the change is complex, explaining the rationale, decisions made, or potential impacts.
*   **When Useful**: Summaries are particularly useful for retrospective analysis of object-oriented design within changes or for large overhaul changes across multiple files. They are less critical for minor syntax fixes or reverting changes.
