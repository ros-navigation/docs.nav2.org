# Replaces edit_url for a specific page based on metadata in the header.
# Used to redirect shared pages to Rolling branch on data edit attempts.
# Each of these pages must contain the following override in the header:
# ---
# edit_uri: https://github.com/ros-navigation/docs.nav2.org/tree/rolling/docs/
# ---

def on_page_context(context, page, config, **kwargs):
    if 'edit_uri' in page.meta:
        page.edit_url = page.meta['edit_uri'] + page.file.src_uri
