// macOS: the app is made the active one, so its main window comes over the Finder or the
// Terminal it was started from.

#import <Cocoa/Cocoa.h>

namespace sicdm {

void bringAppToFrontMac() {
    [NSApp activateIgnoringOtherApps:YES];
    for (NSWindow* w in [NSApp windows])
        if ([w isVisible] && [w canBecomeMainWindow]) {
            [w makeKeyAndOrderFront:nil];
            break;
        }
}

} // namespace sicdm
